#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"
#include "app_adc_sample.h"
#include "app_timer.h"
#include "fsl_opamp.h"
#include "fsl_spc.h"
#include "app_encoder.h"
#include "hipnuc.h"
#include <stdio.h>
#include "app_sampler.h"

#define DEMO_OPAMP_COMP_CAP       kOPAMP_FitGain2x
#define DEMO_OPAMP_BIAS_CURRENT   kOPAMP_NoChange

/*! @brief Configure OPAMP modules */
static void DEMO_DoOpampConfig(void)
{
    opamp_config_t config;

    SPC_EnableActiveModeAnalogModules(SPC0, (kSPC_controlOpamp0 | kSPC_controlOpamp1));
    
    OPAMP_GetDefaultConfig(&config);
    config.compCap     = DEMO_OPAMP_COMP_CAP;
    config.biasCurrent = DEMO_OPAMP_BIAS_CURRENT;
    config.enable = true;
    
    OPAMP_Init(OPAMP0, &config);
    OPAMP_Init(OPAMP1, &config);
    
    SPC_SetActiveModeBandgapModeConfig(SPC0, kSPC_BandgapEnabledBufferEnabled);
    
    OPAMP_Enable(OPAMP0, true);
    OPAMP_Enable(OPAMP1, true);
}

/* Initialize SysTick for 10Hz interrupt */
void SysTick_Init(void)
{
    uint32_t systemClock = CLOCK_GetCoreSysClkFreq();
    SysTick_Config(systemClock / 20);
}


/* Setup FRO clock trimming for high-speed accuracy */
void app_FircAutoTrim(uint32_t osc_frq)
{
    CLOCK_SetupExtClocking(osc_frq);              // Enable the 8MHz external crystal
}


int main(void)
{
    char ch;
    uint32_t freq;
    adc_sampling_mode_t adc_mode;

    /* Init board hardware */
    BOARD_InitHardware();
    
    app_FircAutoTrim(8*1000*1000);

    /* Initialize high precision timer */
    TIMER_Init();
    
    SysTick_Init();

    printf("=== System Clocks ===\r\n");
    freq = CLOCK_GetFreq(kCLOCK_CoreSysClk);
    printf("Core Clock:    %u Hz (%u MHz)\r\n", freq, freq / 1000000U);

    freq = CLOCK_GetFreq(kCLOCK_FroHf);
    printf("FRO_HF:        %u Hz (%u MHz)\r\n", freq, freq / 1000000U);

    freq = CLOCK_GetFreq(kCLOCK_FroHfDiv);
    printf("FRO_HF_DIV:    %u Hz (%u MHz)\r\n", freq, freq / 1000000U);
    
    /* Initialize ADC and OPAMP */
    DEMO_DoOpampConfig();
    
    /* ========== Mode Selection ========== */
    printf("\r\n=== Select ADC Sampling Mode ===\r\n");
    printf("1: Normal Mode (OPAMP outputs only, 2ch)\r\n");
    printf("2: Debug Mode (All 6 channels: INP, INN, OUT)\r\n");
    printf("3: Auto Calibration (OPAMP outputs only, 2ch)\r\n");
    printf("Select: ");
    
    ch = GETCHAR();
    printf("%c\r\n", ch);
    
    /* Determine ADC mode */
    if (ch == '2') {
        adc_mode = ADC_MODE_FULL_DEBUG;
    } else if (ch == '3') {
        adc_mode = ADC_MODE_CALIBRATION;
    } else {
        adc_mode = ADC_MODE_OUTPUT_ONLY;
        ch = '1';  // Force to mode 1
    }
    
    adc_init(adc_mode);
    
    /* ========== Auto Calibration Mode ========== */
    if (ch == '3') {
        printf("\r\n=== Auto Calibration ===\r\n");
        printf("Rotate encoder slowly for one full circle...\r\n");
        printf("Press any key to start...\r\n");
        
        GETCHAR();
        
        uint16_t sin_min = 65535, sin_max = 0;
        uint16_t cos_min = 65535, cos_max = 0;

        for (int i = 0; i < 300000; i++) {
            adc_sample_result_t result = adc_read();
            
            if (result.opamp0_out < sin_min) sin_min = result.opamp0_out;
            if (result.opamp0_out > sin_max) sin_max = result.opamp0_out;
            if (result.opamp1_out < cos_min) cos_min = result.opamp1_out;
            if (result.opamp1_out > cos_max) cos_max = result.opamp1_out;
            
            if ((i % 1000) == 0) {
                printf("  Progress: %d/300000\r\n", i);
            }
            
            SDK_DelayAtLeastUs(100, CLOCK_GetFreq(kCLOCK_CoreSysClk));
        }

        encoder_calibrate(sin_min, sin_max, cos_min, cos_max);
        printf("=== Calibration Complete ===\r\n");
        printf("OPAMP0_OUT (Sin): [%d, %d]\r\n", sin_min, sin_max);
        printf("OPAMP1_OUT (Cos): [%d, %d]\r\n\r\n", cos_min, cos_max);
        
        ch = '1';  // Switch to normal mode after calibration
    }
    
    
    sampler_init(10*1000);
    sampler_start();
    
    /* ========== Normal/Debug Mode ========== */
    if (ch == '1' || ch == '2') {

        uint8_t tx_buf[128];
        nl_t acc[3] = {0, 0, 0};
        nl_t gyr[3] = {0, 0, 0};
        nl_t quat[4] = {1, 0, 0, 0};
        nl_t mag[3];
        
        uint32_t frame_count = 0;
        
        while (1) {
            adc_sample_result_t result;
            encoder_result_t encoderResult;
            
            sampler_copy_latest(&encoderResult);
            sampler_copy_latest_raw(&result);
            
            /* Convert to voltage */
            float v_op0_out = (result.opamp0_out * 3.3f) / 65536.0f;
            float v_op1_out = (result.opamp1_out * 3.3f) / 65536.0f;
            
            /* Pack common data */
            mag[0] = v_op0_out;                    // Sin output
            mag[1] = v_op1_out;                    // Cos output
            
            extern float adc_sample_time_us;
            
            mag[2] = adc_sample_time_us;
            
            if (adc_get_mode() == ADC_MODE_FULL_DEBUG) {
                /* Debug mode: calculate differential voltages */
                int16_t diff0 = (int16_t)result.opamp0_inp - (int16_t)result.opamp0_inn;
                int16_t diff1 = (int16_t)result.opamp1_inp - (int16_t)result.opamp1_inn;
                float v_diff0 = (diff0 * 3.3f) / 65536.0f;
                float v_diff1 = (diff1 * 3.3f) / 65536.0f;
                
                acc[0] = v_diff0;                      // OPAMP0 differential
                acc[1] = v_diff1;                      // OPAMP1 differential
                acc[2] = v_op0_out - v_op1_out;        // Output difference
                
                
                /* Print debug info every 10 frames */
//                    printf("Ang:%6.2f� | S/C:[%+.3f,%+.3f] | Diff:[%+.3f,%+.3f] | Mag:%.3f | %5.1fus\r\n",
//                           encoderResult.angle_deg,
//                           encoderResult.sin_norm,
//                           encoderResult.cos_norm,
//                           v_diff0,
//                           v_diff1,
//                           encoderResult.magnitude,
//                           elapsed_time_us);
                }
            
            /* Build and send HiPNUC frame */
            int frame_len = bin_hi91data(
                tx_buf, 0, 0, acc, gyr, quat, mag, 0, 0, encoderResult.angle_deg, encoderResult.turns, 0
            );
            
            for (int i = 0; i < frame_len; i++) {
                PUTCHAR(tx_buf[i]);
            }
            
            frame_count++;
            SDK_DelayAtLeastUs(5000, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // 200Hz
        }
    }
    
    if (ch == 'r') {
        NVIC_SystemReset();
    }
}

void SysTick_Handler(void)
{
    // GPIO_PortToggle(LED_GPIO, 1u << LED_PIN);
}

void HardFault_Handler(void)
{
    printf("HardFault_Handler\r\n");
    while(1);
}
