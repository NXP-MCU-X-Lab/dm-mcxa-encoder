#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app_adc.h"
#include "app_timer.h"

#include "app_encoder.h"
#include "hipnuc.h"
#include <stdio.h>
#include "app_sampler.h"
#include "hardware_init.h"

#define SAMPLE_FRQ          (10*1000)


static adc_sample_result_t adc_result;
static encoder_result_t encoder_result;
static uint8_t timer_evt;
static volatile uint32_t systick_counter = 0;  // SysTick ???


/* Initialize SysTick for 10Hz interrupt */
void SysTick_Init(void)
{
    uint32_t systemClock = CLOCK_GetCoreSysClkFreq();
    SysTick_Config(systemClock / 100);
}





/* Non-blocking getchar with timeout */
static int getchar_timeout(uint32_t timeout_ms)
{
    LPUART_Type *base = LPUART2;
    uint32_t start = systick_counter;
    uint32_t timeout_ticks = timeout_ms / 10;  // Convert ms to 10ms ticks
    
    while ((systick_counter - start) < timeout_ticks) {
        /* Check if RX data register is full */
        if ((base->STAT & LPUART_STAT_RDRF_MASK) != 0U) {
            return (int)(base->DATA & 0xFFU);
        }
    }
    return -1;  // Timeout
}


int main(void)
{
    int ch;
    uint32_t freq;
    adc_sampling_mode_t adc_mode;

    /* Init board hardware */
    BOARD_InitHardware();
    
    CLOCK_SetupExtClocking(8*1000*1000);              // Enable the 8MHz external crystal

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
    
    /* ========== Mode Selection with 3s Timeout ========== */
    printf("\r\n=== Select ADC Sampling Mode (3s timeout) ===\r\n");
    printf("1: Normal Mode (OPAMP outputs only, 2ch)\r\n");
    printf("2: Debug Mode (All 6 channels: INP, INN, OUT)\r\n");
    printf("3: Auto Calibration (OPAMP outputs only, 2ch)\r\n");
    printf("Select: ");
    
    ch = getchar_timeout(3000);  // 3 second timeout
    
    if (ch == -1) {
        printf("timeout\r\n");
        ch = '1';  // Default to normal mode
        printf("Auto-selecting Normal Mode\r\n");
    } else {
        printf("%c\r\n", (char)ch);
    }
    
    /* Determine ADC mode */
    if (ch == '2') {
        adc_mode = ADC_MODE_FULL_DEBUG;
    } else if (ch == '3') {
        adc_mode = ADC_MODE_CALIBRATION;
    } else {
        adc_mode = ADC_MODE_OUTPUT_ONLY;
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
    

    sampler_init(SAMPLE_FRQ);
    sampler_start();
    encoder_set_direction(-1);
   
    uint8_t tx_buf[128];
    nl_t acc[3] = {0, 0, 0};
    nl_t gyr[3] = {0, 0, 0};
    nl_t quat[4] = {1, 0, 0, 0};
    nl_t mag[3];
    
    while (1) {
        if(timer_evt == 1) {
            timer_evt = 0;
           
            sampler_copy_latest(&encoder_result);
            sampler_copy_latest_raw(&adc_result);
            mag[0] = adc_result.opamp0_out_voltage;
            mag[1] = adc_result.opamp1_out_voltage;
            mag[2] = encoder_result.angle_counts;
            int frame_len = bin_hi91data(
                tx_buf, 0, 0, acc, gyr, quat, mag, 0, 0, encoder_result.angle_deg, encoder_result.turns, 0
            );
            
            for (int i = 0; i < frame_len; i++) {
                PUTCHAR(tx_buf[i]);
            }
        }
    }
    
}

void SysTick_Handler(void)
{
    systick_counter++;
    timer_evt = 1;
}

void HardFault_Handler(void)
{
    printf("HardFault_Handler\r\n");
    while(1);
}
