#include "app_def.h"
#include "app_usr_data.h"
#include "app_wave_est.h"
#include "hipnuc_reg.h"

#define DBG_TAG    "app.hipnuc_reg"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>


static const uint32_t CAN_BAUD_TABLE[] = {
    1000*1000, 800*1000, 500*1000, 250*1000, 125*1000, 
    100*1000, 50*1000, 20*1000, 10*1000, 500*1000
};

#define CAN_BAUD_TABLE_SIZE (sizeof(CAN_BAUD_TABLE)/sizeof(CAN_BAUD_TABLE[0]))
    
static int16_t can_baud_to_code(uint32_t baud) {
    for(int i = 0; i < CAN_BAUD_TABLE_SIZE; i++) {
        if(baud == CAN_BAUD_TABLE[i]) return i;
    }
    return 2;
}

static uint32_t can_code_to_baud(int16_t code) {
    return (code >= 0 && code < CAN_BAUD_TABLE_SIZE) ? 
           CAN_BAUD_TABLE[code] : CAN_BAUD_TABLE[2];
}


static const uint32_t UART_BAUD_TABLE[] = {
    4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
};
#define UART_BAUD_TABLE_SIZE (sizeof(UART_BAUD_TABLE)/sizeof(UART_BAUD_TABLE[0]))

static int16_t uart_baud_to_code(uint32_t baud) {
    for(int i = 0; i < UART_BAUD_TABLE_SIZE; i++) {
        if(baud == UART_BAUD_TABLE[i]) return i;
    }
    return 5;
}

static uint32_t uart_code_to_baud(int16_t code) {
    return (code >= 0 && code < UART_BAUD_TABLE_SIZE) ? 
           UART_BAUD_TABLE[code] : UART_BAUD_TABLE[1];
}

static void hipnuc_write_reg_att_rst(int16_t val)
{
    rt_setting_t app = rt_setting_find(SETTING_APP_NAME);
    att_t att;
    nl_t qofs[4];
    
    q2att(eskfsvr.ins.q, &att);

    switch(val)
    {
        case 0:  /* OBJECT: ALL EUL=0 */
            att.pitch = 0; 
            att.roll = 0; 
            att.yaw = 0;
            break;
            
        case 2: /* ALIGNMENT P/R = 0 */
            att.pitch = 0;
            att.roll = 0;
            break;
            
        case 3: /* auto simple cal */
        {
            // Check if device is nearly horizontal (pitch and roll < 5 degrees)
            nl_t horizontal_threshold = 5.0f * D2R;
            
            if (fabsf(att.pitch) > horizontal_threshold || 
                fabsf(att.roll) > horizontal_threshold) {
                return; // Device not horizontal enough
            }
            
            // Clear pitch and roll
            att.pitch = 0;
            
            // Check if device is upside down (roll close to -180 degrees)
            if (fabsf(att.roll) > (M_PI - horizontal_threshold)) {
                att.roll = M_PI;  // Set to exactly 180
            } else {
                att.roll = 0;     // Set to 0
            }
            break;
        }
        
        case 1:
        case 4: /* clear yaw, not save */
        {
            att.pitch = 0; 
            att.roll = 0;
            
            nl_adj_att(eskfsvr.ins.q, &att, qofs);
            qconj2(qofs);
            vcopy(eskfsvr.ins.q, qofs, 4);
            nl_zaru_force_motion(&eskfsvr.zaru_det);
            return;
        }
        case 5: /* restore */
            
            break;
        
        default:
            return;
    }

    nl_adj_att(eskfsvr.ins.q, &att, qofs);
    app_usr_data_set_quat_offset(qofs);
    rt_param_set_float_array("QOFS", qofs, 4, app);
    nl_zaru_force_motion(&eskfsvr.zaru_det);
}

static void hipnuc_write_ctl_freset(void)
{
    rt_setting_t app = rt_setting_find(SETTING_APP_NAME);
    
    rt_setting_reset(app);
    
    hipnuc_wr_reg(HIREG_ADDR_URFR, 0);

    /* disable data output when using 485 mode */
    char pname[32];
    if(bl_get_product_name(pname, sizeof(pname)) == 0) rt_param_set_string("PNAME", pname, app);

    if(strstr(rt_param_get_string("PNAME", app), "-485"))
    {
        rt_param_set_int("CM1/*", 0, app);
    }
    
    /* inclimenater */
    if(strstr(rt_param_get_string("PNAME", app), "HI50"))
    {
        if(strstr((const char*)rt_param_get_string("PNAME", app), "-D1")) rt_param_set_int("EN_INC_2PI", 1, app);
        if(strstr((const char*)rt_param_get_string("PNAME", app), "-D2")) rt_param_set_int("EN_INC_2PI", 0, app);
        
        rt_param_set_int_array_val("J9_TMR", 0, 0, app);
        rt_param_set_int_array_val("J9_TMR", 0, 1, app);
        rt_param_set_int_array_val("J9_TMR", 0, 2, app);
        rt_param_set_int_array_val("J9_TMR", 0, 3, app);
        rt_param_set_int_array_val("J9_TMR", 10, 4, app);
        rt_param_set_int_array_val("J9_TMR", 10, 5, app);
        rt_param_set_int_array_val("J9_TMR", 0, 6, app);
        rt_param_set_int_array_val("J9_TMR", 10, 7, app);
        rt_param_set_int_array_val("J9_TMR", 0, 8, app);
        rt_param_set_int_array_val("J9_TMR", 0, 9, app);
        rt_param_set_int_array_val("J9_TMR", 10, 10, app);

        if(strstr(rt_param_get_string("PNAME", app), "-V"))
        {
            hipnuc_wr_reg(HIREG_ADDR_URFR, 1);
        }
        
        rt_param_set_int("PROFILE", APP_PROFILE_LOW_DYN_INCLINOMETER, app);
    }
    
    
    if(strstr((const char*)rt_param_get_string("PNAME", app), "HI32"))
    {
        rt_param_set_int("PROFILE", APP_PROFILE_INS_VELICHLE, app);
        rt_param_set_int("CM1/*", 0, app);
        rt_param_set_int("CM1/HI81", 10, app);

        rt_param_set_int_array_val("J9_TMR", 20, 0, app);
        rt_param_set_int_array_val("J9_TMR", 20, 1, app);
        rt_param_set_int_array_val("J9_TMR", 20, 2, app);
        rt_param_set_int_array_val("J9_TMR", 20, 3, app);
        rt_param_set_int_array_val("J9_TMR", 20, 4, app);
        rt_param_set_int_array_val("J9_TMR", 20, 5, app);
        rt_param_set_int_array_val("J9_TMR", 20, 6, app);
        rt_param_set_int_array_val("J9_TMR", 20, 7, app);
        rt_param_set_int_array_val("J9_TMR", 0, 8, app);
        rt_param_set_int_array_val("J9_TMR", 0, 9, app);
        rt_param_set_int_array_val("J9_TMR", 0, 10, app);
    }
        
    /* FRESET will not erase caliration data */
    rt_param_set_float_array("GMIS", eskfsvr.imu_mgr.gmis.dat, 9, app);
    rt_param_set_float_array("WMIS", eskfsvr.imu_mgr.wmis.dat, 9, app);
    rt_param_set_float_array("GB", eskfsvr.imu_mgr.calib.gb, 3, app);
    rt_param_set_float_array("WB", eskfsvr.imu_mgr.calib.wb, 3, app);

    // Temperature compensation parameters
    rt_param_set_float_array("GB_TC1", eskfsvr.imu_mgr.calib.gb_tc1, 3, app);
    rt_param_set_float_array("GB_TC2", eskfsvr.imu_mgr.calib.gb_tc2, 3, app);
    rt_param_set_float_array("WB_TC1", eskfsvr.imu_mgr.calib.wb_tc1, 3, app);
    rt_param_set_float_array("WB_TC2", eskfsvr.imu_mgr.calib.wb_tc2, 3, app);
    rt_param_set_float_array("GB_TC1_LOW", eskfsvr.imu_mgr.calib.gb_tc1_low, 3, app);
    rt_param_set_float_array("GB_TC2_LOW", eskfsvr.imu_mgr.calib.gb_tc2_low, 3, app);
    rt_param_set_float_array("WB_TC1_LOW", eskfsvr.imu_mgr.calib.wb_tc1_low, 3, app);
    rt_param_set_float_array("WB_TC2_LOW", eskfsvr.imu_mgr.calib.wb_tc2_low, 3, app);
    rt_param_set_float_array("WS_TC1", eskfsvr.imu_mgr.calib.ws_tc1, 3, app);
    rt_param_set_float_array("WS_TC2", eskfsvr.imu_mgr.calib.ws_tc2, 3, app);

    rt_param_set_float("GB_TC_TEMP", eskfsvr.imu_mgr.calib.gb_cal_temp, app);
    rt_param_set_float("WB_TC_TEMP", eskfsvr.imu_mgr.calib.wb_cal_temp, app);
    
    /* more here, more die to company */
    
    /* SB1, need HI90 protcol */
    if(strstr(rt_param_get_string("PNAME", app), "HI04M0") && strstr(rt_param_get_string("PNAME", app), "-001"))
    {
        rt_param_set_int("CM1/*", 0, app);
        rt_param_set_int("CM1/HI90", 10, app);
    }
    
    
    hipnuc_wr_reg(HIREG_ADDR_CTRL, HIREG_CTRL_SAVECONFIG);
    hipnuc_wr_reg(HIREG_ADDR_CTRL, HIREG_CTRL_REBOOT);
}

static void irq_tmr_reboot(void *parameter) 
{
    rt_hw_cpu_reset();
}


static void hipnuc_write_reg_ctl(int16_t val)
{
    if(val == HIREG_CTRL_SAVECONFIG) {rt_setting_save(rt_setting_find(SETTING_APP_NAME));};
    if(val == HIREG_CTRL_FRESET) hipnuc_write_ctl_freset();
    if(val == HIREG_CTRL_REBOOT) rt_timer_start(rt_timer_create("tmr", irq_tmr_reboot, RT_NULL, rt_tick_from_millisecond(5), RT_TIMER_FLAG_ONE_SHOT));

    if(val == HIREG_CTRL_ENTER_BL)
    {
        rt_setting_t app = rt_setting_find(SETTING_APP_NAME);
        
        blapi_arg_t boot_arg;
        boot_arg.tag = API_BOOT_TAG;
        boot_arg.uart_baud[0] = rt_param_get_int("CM1_BAUD", app);
        boot_arg.uart_baud[1] = rt_param_get_int("CM2_BAUD", app);
        boot_arg.can_baud = rt_param_get_int("CAN1_BAUD", app);
        boot_arg.can_id = rt_param_get_int("CAN_NODE_ID", app);
        boot_arg.timeout_ms = 2000;
        rt_thread_mdelay(10);
        bl_enter_bootloader(&boot_arg);
    }
}

void hipnuc_wr_reg(uint16_t addr, int16_t val)
{
    LOG_D("hipnuc_wr_reg, ADDR:0x%X, val:0x%X\r\n", addr, val);
    
    rt_setting_t app = rt_setting_find(SETTING_APP_NAME);
    
    switch(addr)
    {
        case HIREG_ADDR_CTRL:           hipnuc_write_reg_ctl(val);  break;
        case HIREG_ADDR_MD_ID:          rt_param_set_int("MODBUS_ID", val, app); break;
        
        case HIREG_ADDR_ACC_BW:         rt_param_set_int("ABW", val, app); break;
        case HIREG_ADDR_GYR_BW:         rt_param_set_int("GBW", val, app); break;
        case HIREG_ADDR_INCLI_X_NEG:    rt_param_set_int("EN_INC_X_NEG", val, app); break;
        case HIREG_ADDR_INCLI_Y_NEG:    rt_param_set_int("EN_INC_Y_NEG", val, app); break;
        case HIREG_ADDR_J9_ID:          rt_param_set_int("CAN_NODE_ID", val, app); break;
        case HIREG_ADDR_J9_EN:          rt_param_set_int("J9_EN", val, app); break;
        case HIREG_ADDR_USR_ACC_FC:     rt_param_set_int("AFC", val, app); break;
        case HIREG_ADDR_USR_GYR_FC:     rt_param_set_int("GFC", val, app); break;
        
        case HIREG_ADDR_UART1_BAUD:     rt_param_set_int("CM1_BAUD", uart_code_to_baud(val), app); break;
        case HIREG_ADDR_CAN1_BAUD:      rt_param_set_int("CAN1_BAUD", can_code_to_baud(val), app);  break;
        case HIREG_ADDR_CO0_ID:         rt_param_set_int("CAN_NODE_ID", val, app); break;
        
        case HIREG_ADDR_J9_TMR_LAT_H:   rt_param_set_int_array_val("J9_TMR", val, 0, app); break;
        case HIREG_ADDR_J9_TMR_HGT_H:   rt_param_set_int_array_val("J9_TMR", val, 1, app); break;
        case HIREG_ADDR_J9_TMR_SOLQ:    rt_param_set_int_array_val("J9_TMR", val, 2, app); break;
        case HIREG_ADDR_J9_TMR_YYMM:    rt_param_set_int_array_val("J9_TMR", val, 3, app); break;
        case HIREG_ADDR_J9_TMR_ACC:     rt_param_set_int_array_val("J9_TMR", val, 4, app); break;
        case HIREG_ADDR_J9_TMR_GYR:     rt_param_set_int_array_val("J9_TMR", val, 5, app); break;
        case HIREG_ADDR_J9_TMR_R_H:     rt_param_set_int_array_val("J9_TMR", val, 6, app); break;
        case HIREG_ADDR_J9_TMR_YAW:     rt_param_set_int_array_val("J9_TMR", val, 7, app); break;
        case HIREG_ADDR_J9_TMR_TEMP:    rt_param_set_int_array_val("J9_TMR", val, 8, app); break;
        case HIREG_ADDR_J9_TMR_VEL_ENU: rt_param_set_int_array_val("J9_TMR", val, 9, app); break;
        case HIREG_ADDR_J9_TMR_INCLI:   rt_param_set_int_array_val("J9_TMR", val, 10, app); break;
        case HIREG_ADDR_J9_TMR_MAG:     rt_param_set_int_array_val("J9_TMR", val, 11, app); break;
        case HIREG_ADDR_J9_TMR_QUAT:    rt_param_set_int_array_val("J9_TMR", val, 12, app); break;
        
        case HIREG_ADDR_PROFILE:        rt_param_set_int("PROFILE", val, app); break;
        case HIREG_ADDR_IMU_COORD:      rt_param_set_int("COORD", val, app); break;
        case HIREG_ADDR_ATT_RST:        hipnuc_write_reg_att_rst(val); break;
        case HIREG_ADDR_URFR:
        {
            rt_param_set_int("URFR", val, app);
            rt_setting_save(app);
            break;
        }
        case HIREG_ADDR_EN_ASYNC_VCOM1: {rt_vcom_enable(rt_vcom_find(VCOM1_NAME), val); break;}
        default:
            break;
    }
}


void hipnuc_rd_reg(uint16_t reg_addr, int16_t* buf, uint8_t len)
{
    int i;
    rt_setting_t app = rt_setting_find(SETTING_APP_NAME);
    LOG_D("hipnuc_rd_reg, ADDR:0x%X, len:%d\r\n", reg_addr, len);
    
    topic_wave_data_t *wave_data = RT_NULL;
    topic_usr_data_t *usr_data = RT_NULL;
    
    rt_topic_t topic_usr_data = rt_topic_find("usr_data");

    if(topic_usr_data)
    {
        usr_data = (topic_usr_data_t*)rt_topic_get_data_ptr(topic_usr_data);    
    }

    rt_topic_t topic_usr_wave = rt_topic_find("usr_wave");
    
    if(topic_usr_wave)
    {
        wave_data = (topic_wave_data_t*)rt_topic_get_data_ptr(topic_usr_wave);
    }

    // Pre-validate data pointers
    bool usr_data_valid = (usr_data != RT_NULL);
    bool wave_data_valid = (wave_data != RT_NULL);

    for(i=0; i<len; i++)
    {
        switch(reg_addr + i)
        {
            case HIREG_ADDR_PROFILE: buf[i] = rt_param_get_int("PROFILE", app); break;
            case HIREG_ADDR_IMU_COORD: buf[i] = rt_param_get_int("COORD", app); break;
            case HIREG_ADDR_ACC_BW:   buf[i] = rt_param_get_int("ABW", app); break;
            case HIREG_ADDR_UART1_BAUD: buf[i] = uart_baud_to_code(rt_param_get_int("CM1_BAUD", app)); break;
            case HIREG_ADDR_MD_ID: buf[i] = rt_param_get_int("MODBUS_ID", app); break;
            case HIREG_ADDR_URFR: buf[i] = rt_param_get_int("URFR", app); break;
            case HIREG_ADDR_MAIN_STATUS: buf[i] = usr_data->main_status; break;
            case HIREG_ADDR_LAT_H: buf[i] = ((int32_t)(eskfsvr.ins.lla[0]*R2D*10000000)) >> 16; break;
            case HIREG_ADDR_LAT_L: buf[i] = ((int32_t)(eskfsvr.ins.lla[0]*R2D*10000000)) >> 0; break;
            case HIREG_ADDR_LON_H: buf[i] = ((int32_t)(eskfsvr.ins.lla[1]*R2D*10000000)) >> 16; break;
            case HIREG_ADDR_LON_L: buf[i] = ((int32_t)(eskfsvr.ins.lla[1]*R2D*10000000)) >> 0; break;
            
            // IMU data - check usr_data validity
            case HIREG_ADDR_ACCX: buf[i] = usr_data_valid ? usr_data->acc[0]/GRAVITY*(32768/16) : 0; break;
            case HIREG_ADDR_ACCY: buf[i] = usr_data_valid ? usr_data->acc[1]/GRAVITY*(32768/16) : 0; break;
            case HIREG_ADDR_ACCZ: buf[i] = usr_data_valid ? usr_data->acc[2]/GRAVITY*(32768/16) : 0; break;
            case HIREG_ADDR_GYRX: buf[i] = usr_data_valid ? usr_data->gyr[0]*R2D*(16.384) : 0; break;
            case HIREG_ADDR_GYRY: buf[i] = usr_data_valid ? usr_data->gyr[1]*R2D*(16.384) : 0; break;
            case HIREG_ADDR_GYRZ: buf[i] = usr_data_valid ? usr_data->gyr[2]*R2D*(16.384) : 0; break;
            case HIREG_ADDR_MAGX: buf[i] = usr_data_valid ? usr_data->mag[0]*(32768/1000) : 0; break;
            case HIREG_ADDR_MAGY: buf[i] = usr_data_valid ? usr_data->mag[1]*(32768/1000) : 0; break;
            case HIREG_ADDR_MAGZ: buf[i] = usr_data_valid ? usr_data->mag[2]*(32768/1000) : 0; break;

            // Attitude data - check usr_data validity
            case HIREG_ADDR_R_H: buf[i] = usr_data_valid ? ((int32_t)(usr_data->att.roll*1000*R2D) >> 16) & 0xFFFF : 0; break;
            case HIREG_ADDR_R_L: buf[i] = usr_data_valid ? ((int32_t)(usr_data->att.roll*1000*R2D) >> 0) & 0xFFFF : 0; break;
            case HIREG_ADDR_P_H: buf[i] = usr_data_valid ? ((int32_t)(usr_data->att.pitch*1000*R2D) >> 16) & 0xFFFF : 0; break;
            case HIREG_ADDR_P_L: buf[i] = usr_data_valid ? ((int32_t)(usr_data->att.pitch*1000*R2D) >> 0) & 0xFFFF : 0; break;
            case HIREG_ADDR_Y_H: buf[i] = usr_data_valid ? ((int32_t)(usr_data->att.yaw*1000*R2D) >> 16) & 0xFFFF : 0; break;
            case HIREG_ADDR_Y_L: buf[i] = usr_data_valid ? ((int32_t)(usr_data->att.yaw*1000*R2D) >> 0) & 0xFFFF : 0; break;
            case HIREG_ADDR_TEMP: buf[i] = usr_data_valid ? usr_data->temp*100 : 0; break;
            case HIREG_ADDR_PRS_H: buf[i] = usr_data_valid ? (((int32_t)usr_data->prs*100) >> 16) : 0; break;
            case HIREG_ADDR_PRS_L: buf[i] = usr_data_valid ? (((int32_t)usr_data->prs*100) >> 0) : 0; break;
            case HIREG_ADDR_Q0: buf[i] = usr_data_valid ? usr_data->q[0]*10000 : 0; break;
            case HIREG_ADDR_Q1: buf[i] = usr_data_valid ? usr_data->q[1]*10000 : 0; break;
            case HIREG_ADDR_Q2: buf[i] = usr_data_valid ? usr_data->q[2]*10000 : 0; break;
            case HIREG_ADDR_Q3: buf[i] = usr_data_valid ? usr_data->q[3]*10000 : 0; break;

            // Inclination data - check usr_data validity
            case HIREG_ADDR_INCLI_X: buf[i] = usr_data_valid ? usr_data->incli_x*R2D*90.909 : 0; break;
            case HIREG_ADDR_INCLI_Y: buf[i] = usr_data_valid ? usr_data->incli_y*R2D*90.909 : 0; break;
            
            // System time
            case HIREG_ADDR_CPUTIME_MS_H: buf[i] = ((rt_tick_get_millisecond()) >> 16) & 0xFFFF; break;
            case HIREG_ADDR_CPUTIME_MS_L: buf[i] = ((rt_tick_get_millisecond()) >> 0) & 0xFFFF; break;
            
            // Wave data - check wave_data validity
            case HIREG_ADDR_HEAVE: buf[i] = wave_data_valid ? wave_data->displacement_ap[2] * 100 : 0; break;
            case HIREG_ADDR_SURGE: buf[i] = wave_data_valid ? wave_data->displacement_ap[1] * 100 : 0; break;
            case HIREG_ADDR_SWAY: buf[i] = wave_data_valid ? wave_data->displacement_ap[0] * 100 : 0; break;
            case HIREG_ADDR_HEAVE_FRQ: buf[i] = wave_data_valid ? wave_data->estimated_freq[2] * 100 : 0; break;
            case HIREG_ADDR_SURGE_FRQ: buf[i] = wave_data_valid ? wave_data->estimated_freq[1] * 100 : 0; break;
            case HIREG_ADDR_SWAY_FRQ: buf[i] = wave_data_valid ? wave_data->estimated_freq[0] * 100 : 0; break;
            
            /* INFO */
            case HIREG_ADDR_PNAME0: buf[i] = (rt_param_get_string("PNAME", app)[0]<<8) + rt_param_get_string("PNAME", app)[1]; break;
            case HIREG_ADDR_PNAME1: buf[i] = (rt_param_get_string("PNAME", app)[2]<<8) + rt_param_get_string("PNAME", app)[3]; break;
            case HIREG_ADDR_PNAME2: buf[i] = (rt_param_get_string("PNAME", app)[4]<<8) + rt_param_get_string("PNAME", app)[5]; break;
            case HIREG_ADDR_PNAME3: buf[i] = (rt_param_get_string("PNAME", app)[6]<<8) + rt_param_get_string("PNAME", app)[7]; break;
            case HIREG_ADDR_PNAME4: buf[i] = (rt_param_get_string("PNAME", app)[8]<<8) + rt_param_get_string("PNAME", app)[9]; break;
            case HIREG_ADDR_PNAME5: buf[i] = (rt_param_get_string("PNAME", app)[10]<<8) + rt_param_get_string("PNAME", app)[11]; break;
            case HIREG_ADDR_PNAME6: buf[i] = (rt_param_get_string("PNAME", app)[12]<<8) + rt_param_get_string("PNAME", app)[13]; break;
            case HIREG_ADDR_PNAME7: buf[i] = (rt_param_get_string("PNAME", app)[14]<<8) + rt_param_get_string("PNAME", app)[15]; break;
            case HIREG_ADDR_SW_VER: buf[i] = APP_VERSION; break;
            case HIREG_ADDR_BL_VER: buf[i] = bl_get_version(); break;
            case HIREG_ADDR_SN0: buf[i] = (eskfsvr.uuid[0] >> 16) & 0xFFFF; break;
            case HIREG_ADDR_SN1: buf[i] = (eskfsvr.uuid[0] >> 0) & 0xFFFF; break;
            case HIREG_ADDR_SN2: buf[i] = (eskfsvr.uuid[1] >> 16) & 0xFFFF; break;
            case HIREG_ADDR_SN3: buf[i] = (eskfsvr.uuid[1] >> 0) & 0xFFFF; break;
            
            // Configuration parameters
            case HIREG_ADDR_INCLI_X_NEG: buf[i] = rt_param_get_int("EN_INC_X_NEG", app); break;
            case HIREG_ADDR_INCLI_Y_NEG: buf[i] = rt_param_get_int("EN_INC_Y_NEG", app); break;
            case HIREG_ADDR_CO0_ID: buf[i] = rt_param_get_int("CAN_NODE_ID", app); break;
            case HIREG_ADDR_J9_ID: buf[i] = rt_param_get_int("CAN_NODE_ID", app); break;
            case HIREG_ADDR_J9_EN: buf[i] = rt_param_get_int("J9_EN", app); break;
            case HIREG_ADDR_CAN1_BAUD: buf[i] = can_baud_to_code(rt_param_get_int("CAN1_BAUD", app)); break;
            
   
            case HIREG_ADDR_USR_ACC_FC: buf[i] = rt_param_get_int("AFC", app); break;
            case HIREG_ADDR_USR_GYR_FC: buf[i] = rt_param_get_int("GFC", app); break;
        
            
            // J9 Timer parameters
            case HIREG_ADDR_J9_TMR_LAT_H: buf[i] = rt_param_get_int_array_val("J9_TMR", 0, app); break; 
            case HIREG_ADDR_J9_TMR_HGT_H: buf[i] = rt_param_get_int_array_val("J9_TMR", 1, app); break; 
            case HIREG_ADDR_J9_TMR_SOLQ: buf[i] = rt_param_get_int_array_val("J9_TMR", 2, app); break; 
            case HIREG_ADDR_J9_TMR_YYMM: buf[i] = rt_param_get_int_array_val("J9_TMR", 3, app); break; 
            case HIREG_ADDR_J9_TMR_ACC: buf[i] = rt_param_get_int_array_val("J9_TMR", 4, app); break; 
            case HIREG_ADDR_J9_TMR_GYR: buf[i] = rt_param_get_int_array_val("J9_TMR", 5, app); break; 
            case HIREG_ADDR_J9_TMR_R_H: buf[i] = rt_param_get_int_array_val("J9_TMR", 6, app); break; 
            case HIREG_ADDR_J9_TMR_YAW: buf[i] = rt_param_get_int_array_val("J9_TMR", 7, app); break; 
            case HIREG_ADDR_J9_TMR_TEMP: buf[i] = rt_param_get_int_array_val("J9_TMR", 8, app); break; 
            case HIREG_ADDR_J9_TMR_VEL_ENU: buf[i] = rt_param_get_int_array_val("J9_TMR", 9, app); break; 
            case HIREG_ADDR_J9_TMR_INCLI: buf[i] = rt_param_get_int_array_val("J9_TMR", 10, app); break; 
            case HIREG_ADDR_J9_TMR_MAG: buf[i] = rt_param_get_int_array_val("J9_TMR", 11, app); break; 
            case HIREG_ADDR_J9_TMR_QUAT: buf[i] = rt_param_get_int_array_val("J9_TMR", 12, app); break; 
            
            case HIREG_ADDR_MBX: buf[i] = eskfsvr.imu_mgr.calib.mb[0] * 100; break;
            case HIREG_ADDR_MBY: buf[i] = eskfsvr.imu_mgr.calib.mb[1] * 100; break;
            case HIREG_ADDR_MBZ: buf[i] = eskfsvr.imu_mgr.calib.mb[2] * 100; break;
                

            default:
                buf[i] = 0;
                break;
        }
    }
}



