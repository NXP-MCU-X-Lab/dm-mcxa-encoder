#include "app_def.h"


#define DBG_TAG    "app.CAL"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>


#define CAL_STILL_NGYR_THR  (3.2 * D2R)
#define ACC_CAL_LIMIT (0.2F * GRAVITY)
#define ACC_CAL_SAMPLE_CNT (2000)

static nl_t acc_sum[3];
static nl_t gyr_sum[3];
static uint32_t gyr_sum_cnt;
static uint32_t acc_sum_cnt;
        
int ACC_CAL(rt_cli_t cli, int argc, char *argv[]);
int GYR_CAL(rt_cli_t cli, int argc, char *argv[]);
int FCAL_TEMP(rt_cli_t cli, int argc, char *argv[]);


int FCAL(rt_cli_t cli, int argc, char **argv)
{
    nl_imu_mgr_t *imu_mgr = &eskfsvr.imu_mgr;
    int ret = RT_ERROR;
    double dat[9];
    
    rt_setting_t app = rt_setting_find(SETTING_APP_NAME);
    
    if (!rt_strcmp("RST", argv[1]))
    {
        rt_setting_reset_param(app, "GMIS");
        rt_setting_reset_param(app, "WMIS");
        rt_setting_reset_param(app, "GB");
        rt_setting_reset_param(app, "WB");
        rt_setting_save(app);
        return RT_EOK;
    }

    if (!rt_strcmp("WMIS", argv[1]))
    {
        if (sscanf(argv[2], "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &dat[0], &dat[1], &dat[2], &dat[3], &dat[4], &dat[5], &dat[6], &dat[7], &dat[8]) == 9)
        {
            imu_mgr->wmis.dat[0] = dat[0];
            imu_mgr->wmis.dat[1] = dat[1];
            imu_mgr->wmis.dat[2] = dat[2];
            imu_mgr->wmis.dat[3] = dat[3];
            imu_mgr->wmis.dat[4] = dat[4];
            imu_mgr->wmis.dat[5] = dat[5];
            imu_mgr->wmis.dat[6] = dat[6];
            imu_mgr->wmis.dat[7] = dat[7];
            imu_mgr->wmis.dat[8] = dat[8];
            ret = RT_EOK;
        }
    }

    if (!rt_strcmp("GMIS", argv[1]))
    {
        if (sscanf(argv[2], "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &dat[0], &dat[1], &dat[2], &dat[3], &dat[4], &dat[5], &dat[6], &dat[7], &dat[8]) == 9)
        {
            imu_mgr->gmis.dat[0] = dat[0];
            imu_mgr->gmis.dat[1] = dat[1];
            imu_mgr->gmis.dat[2] = dat[2];
            imu_mgr->gmis.dat[3] = dat[3];
            imu_mgr->gmis.dat[4] = dat[4];
            imu_mgr->gmis.dat[5] = dat[5];
            imu_mgr->gmis.dat[6] = dat[6];
            imu_mgr->gmis.dat[7] = dat[7];
            imu_mgr->gmis.dat[8] = dat[8];
            ret = RT_EOK;
        }
    }
    
    if (!rt_strcmp("GB", argv[1]))
    {
        if (sscanf(argv[2], "%lf,%lf,%lf", &dat[0], &dat[1], &dat[2]) == 3)
        {
            imu_mgr->calib.gb[0] = dat[0];
            imu_mgr->calib.gb[1] = dat[1];
            imu_mgr->calib.gb[2] = dat[2];
            ret = RT_EOK;
        }
    }

    if (!rt_strcmp("MB", argv[1]))
    {
        if (sscanf(argv[2], "%lf,%lf,%lf", &dat[0], &dat[1], &dat[2]) == 3)
        {
            imu_mgr->calib.mb[0] = dat[0];
            imu_mgr->calib.mb[1] = dat[1];
            imu_mgr->calib.mb[2] = dat[2];
            rt_param_set_float_array("MB", imu_mgr->calib.mb, 3, app);
            ret = RT_EOK;
        }
    }
    
    if (!rt_strcmp("WB", argv[1]))
    {
        if (sscanf(argv[2], "%lf,%lf,%lf", &dat[0], &dat[1], &dat[2]) == 3)
        {
            imu_mgr->calib.wb[0] = dat[0]*D2R;
            imu_mgr->calib.wb[1] = dat[1]*D2R;
            imu_mgr->calib.wb[2] = dat[2]*D2R;
            ret = RT_EOK;
        }
        else if (argc >= 3 && !rt_strcmp("AUTO", argv[2]))
        {
            nl_t gyr_avg_raw[3];
            uint32_t duration_s = 20;
            
            if(argc == 4) sscanf(argv[3], "%d", &duration_s);
            
            
            v3fill(gyr_avg_raw, 0);
            rt_kprintf("BEGIN WB AUTO\n");
            
            uint32_t now = nl_get_sys_ms();
            uint32_t count  = 0;
            
            while(nl_get_sys_ms() - now < (duration_s*1000))
            {
                v3add2(gyr_avg_raw, imu_mgr->buffer.raw.gyr);
                nl_mdelay(10);
                count++;
            }
            
            v3scale(imu_mgr->calib.wb, gyr_avg_raw, 1.0 / count);
            imu_mgr->calib.wb_cal_temp = imu_mgr->buffer.cal.temp;
            rt_kprintf("WB(deg):%.3f %.3f %.3f\n", imu_mgr->calib.wb[0] * R2D, imu_mgr->calib.wb[1] * R2D, imu_mgr->calib.wb[2] * R2D);
            rt_param_set_float("WB_TC_TEMP", imu_mgr->buffer.cal.temp, app);
            ret = RT_EOK;
        }
    }

    uint8_t original_coord = imu_mgr->en_enu2ned;
    
    if (!rt_strcmp("FCA", argv[1]))
    {
        eskfsvr.en_factory_mode = 1;
        ret = ACC_CAL(cli, argc, argv);
        rt_param_set_float("GB_TC_TEMP", imu_mgr->buffer.cal.temp, app);
    }

    if (!rt_strcmp("FCG", argv[1]))
    {
        eskfsvr.en_factory_mode = 1;
        ret = GYR_CAL(cli, argc, argv);
        rt_param_set_float("WB_TC_TEMP", imu_mgr->buffer.cal.temp, app);
    }
    
    imu_mgr->en_enu2ned = original_coord;
    
    /* save data */
    rt_param_set_float_array("GMIS", imu_mgr->gmis.dat, 9, app);
    rt_param_set_float_array("WMIS", imu_mgr->wmis.dat, 9, app);
    rt_param_set_float_array("MMIS", imu_mgr->mmis.dat, 9, app);
    rt_param_set_float_array("GB", imu_mgr->calib.gb, 3, app);
    rt_param_set_float_array("WB", imu_mgr->calib.wb, 3, app);
    rt_setting_save(app);
    
    return ret;
}


static void clear_gyr_sum()
{
    v3fill(gyr_sum, 0);
    gyr_sum_cnt = 0;
}

static void clear_acc_sum()
{
    v3fill(acc_sum, 0);
    acc_sum_cnt = 0;
}

int GYR_CAL(rt_cli_t cli, int argc, char *argv[])
{
    uint8_t gyr_bm = 0, i; /* gyr_bm:7: all complete */
    nl_t omega[3];
    
    /* Variables for duration checking */
    uint32_t axis_stable_time[3] = {0}; /* Start time when each axis becomes stable */
    bool axis_stable[3] = {false};      /* Flag indicating if each axis is stable */
    const uint32_t STABLE_DURATION_MS = 8*1000; /* Required stable duration (ms) */
    const nl_t TARGET_SPEED = 50.0 * D2R; /* Target angular velocity (rad/s) */
    const nl_t SPEED_TARGET_TOLERANCE = 4.0 * D2R; /* Angular velocity tolerance (rad/s) */
    const nl_t SPEED_CROESS_TOLERANCE = 5.0 * D2R;
    
    /* gyroscope */
    m_t *omegas = mcreate(3, 3);
    m_t *b = mcreate(3, 3);

    nl_imu_mgr_t *imu_mgr = &eskfsvr.imu_mgr;
    nl_imu_data_t *raw = &eskfsvr.imu_mgr.buffer.raw;
    
    /* First, measure gyroscope bias */
    clear_gyr_sum();
    
    /* Wait for device to become still and collect bias data */
    nl_stats_basic_t gyr_stats[3]; // Statistics for each axis
    bool is_collecting = false;
    uint32_t collection_start_time = 0;
    const uint32_t STILL_DURATION_MS = 9000; // 8 seconds of still data collection

    // Initialize statistics
    for (i = 0; i < 3; i++) {
        nl_stats_basic_reset(&gyr_stats[i]);
    }

    rt_kprintf("Waiting for still state...\n");

    while ((gyr_bm & 0x08) == 0)
    {
        nl_imu_read_array(&eskfsvr.imu_mgr);
        
        /* Check stillness */
        if (v3norm(raw->gyr) < CAL_STILL_NGYR_THR) {
            if (!is_collecting) {
                // Reset statistics when starting collection
                for (i = 0; i < 3; i++) {
                    nl_stats_basic_reset(&gyr_stats[i]);
                }
                is_collecting = true;
                collection_start_time = nl_get_sys_ms();
                rt_kprintf("Still state detected\n");
            }
            
            // Update statistics
            for (i = 0; i < 3; i++) {
                nl_stats_basic_update(&gyr_stats[i], raw->gyr[i]);
            }
            
            // Check elapsed time
            uint32_t elapsed_time = nl_get_sys_ms() - collection_start_time;
            
            // Check if enough time has passed and we have sufficient samples
            if (elapsed_time >= STILL_DURATION_MS) {
                // Calculate bias
                for (i = 0; i < 3; i++) {
                    imu_mgr->calib.wb[i] = nl_stats_basic_get_mean(&gyr_stats[i]);
                }
                
                msetcol(b, 0, imu_mgr->calib.wb);
                msetcol(b, 1, imu_mgr->calib.wb);
                msetcol(b, 2, imu_mgr->calib.wb);
                
                rt_kprintf("WB: %.3f %.3f %.3f\n", imu_mgr->calib.wb[0] * R2D, imu_mgr->calib.wb[1] * R2D, imu_mgr->calib.wb[2] * R2D);     
                gyr_bm |= 0x08;
            }
        } else {
            // Reset if motion detected
            if (is_collecting) {
                is_collecting = false;
                rt_kprintf("Motion detected\n");
            }
        }
        
        nl_mdelay(2);
    }
        
    /* Start detecting rotation for each axis */
    rt_kprintf("rotate each axis at ~50 dps\n");

    /* Create statistics structures for each axis */
    nl_stats_basic_t axis_stats[3];
    for (i = 0; i < 3; i++) {
        nl_stats_basic_reset(&axis_stats[i]);
    }

    while ((gyr_bm & 0x07) != 0x07) /* Check if all axes calibrated */
    {
        nl_imu_read_array(&eskfsvr.imu_mgr);
        
        /* Check each axis */
        for (i = 0; i < 3; i++)
        {
            if ((gyr_bm & (1 << i)) == 0) /* If axis not yet calibrated */
            {
                nl_t axis_speed = fabs(raw->gyr[i]);
                bool other_axes_stable = true;
                
                /* Check if other axes are near zero */
                for (int j = 0; j < 3; j++) {
                    if (j != i && fabs(raw->gyr[j]) > SPEED_CROESS_TOLERANCE) {
                        other_axes_stable = false;
                        break;
                    }
                }
                
                /* Check if current axis has proper speed and others are stable */
                if (fabs(axis_speed - TARGET_SPEED) < SPEED_TARGET_TOLERANCE && other_axes_stable) {
                    if (!axis_stable[i]) {
                        nl_mdelay(2000);
                        axis_stable[i] = true;
                        axis_stable_time[i] = nl_get_sys_ms();
                        
                        for (int j = 0; j < 3; j++) {
                            nl_stats_basic_reset(&axis_stats[j]);
                        }
                        
                        rt_kprintf("%c: rotation detected\n", 'X' + i);
                    }
                    
                    /* Collect data for all axes */
                    for (int j = 0; j < 3; j++) {
                        nl_stats_basic_update(&axis_stats[j], raw->gyr[j]);
                    }
                    
                    /* Check if stable duration reached */
                    uint32_t stable_time = nl_get_sys_ms() - axis_stable_time[i];
                    if (stable_time >= STABLE_DURATION_MS) {
                        /* Axis calibrated */
                        nl_t mean_speeds[3];
                        
                        /* Calculate mean for all axes */
                        for (int j = 0; j < 3; j++) {
                            mean_speeds[j] = nl_stats_basic_get_mean(&axis_stats[j]);
                        }
                        
                        /* Set the ideal value for the current axis */
                        omega[i] = (mean_speeds[i] > 0) ? TARGET_SPEED : -TARGET_SPEED;
                        
                        /* Record actual measured speeds for all axes */
                        msetcol(omegas, i, mean_speeds);
                        
                        gyr_bm |= (1 << i);
                        rt_kprintf("%c: %.2f dps (cross: %.2f, %.2f)\n", 'X' + i, mean_speeds[i] * R2D, mean_speeds[(i+1)%3] * R2D, mean_speeds[(i+2)%3] * R2D);      
                    }
                    
                } else {
                    /* Reset stable state */
                    axis_stable[i] = false;
                }
            }
        }
        
        /* Calculate calibration matrix when all axes done */
        if ((gyr_bm & 0x07) == 0x07) {
            m_t *m_omega = mcreate(3, 3);
            msetdiag(m_omega, omega);
            msub2(omegas, b);
            minv(omegas);
            mmul(m_omega, omegas, &imu_mgr->wmis);
            rt_kprintf("GYR CAL COMPLETE\n");
            nl_free(m_omega);
            nl_free(omegas);
            nl_free(b);
            gyr_bm |= 0x80;
        }
        nl_mdelay(2);
    }
    
    return RT_EOK;
}


/* factory calibration for acc's mis and bias*/
int ACC_CAL(rt_cli_t cli, int argc, char *argv[])
{
    nl_imu_mgr_t *imu_mgr = &eskfsvr.imu_mgr;
    nl_imu_data_t *raw = &eskfsvr.imu_mgr.buffer.raw;
    
    uint8_t acc_bm = 0; /* acc_bm:7 all complete, gyr_bm:7: all complete */
    nl_t tmp[3], acc_avg[4];
    
    rt_kprintf("FACTORY ACC CALIBRATION START!\n");

    m_t *Y = mcreate(6, 3);
    m_t *W = mcreate(6, 4);
    m_t *X = mcreate(4, 3);
    m_t *XT = mcreate(3, 4);
    m_t *Q = mcreate(4, 4);

    MELEMENT(Y, 0, 2) = GRAVITY;
    MELEMENT(Y, 1, 2) = -GRAVITY;
    MELEMENT(Y, 2, 1) = GRAVITY;
    MELEMENT(Y, 3, 1) = -GRAVITY;
    MELEMENT(Y, 4, 0) = GRAVITY;
    MELEMENT(Y, 5, 0) = -GRAVITY;

    clear_acc_sum();
    vfill(acc_avg, 0, 4);
    vfill(tmp, 0, 3);

    while (1)
    {
        nl_imu_read_array(&eskfsvr.imu_mgr);
        v3add2(acc_sum, raw->acc);
        acc_sum_cnt++;
        
        if (v3norm(raw->gyr) > CAL_STILL_NGYR_THR)
        {
            clear_acc_sum();
        }
            
        /* acc calbiration */
        if (acc_sum_cnt >= ACC_CAL_SAMPLE_CNT && (acc_bm & 0x80) == 0)
        {
            /* calcute mean */
            acc_avg[0] = acc_sum[0] / acc_sum_cnt;
            acc_avg[1] = acc_sum[1] / acc_sum_cnt;
            acc_avg[2] = acc_sum[2] / acc_sum_cnt;

            clear_acc_sum();
            acc_avg[3] = 1;
        }

        if (fabs(acc_avg[0]) < ACC_CAL_LIMIT && fabs(acc_avg[1]) < ACC_CAL_LIMIT && fabs(acc_avg[2] - GRAVITY) < ACC_CAL_LIMIT && (acc_bm & 0x01) == 0)
        {
            acc_bm |= 0x01;
            msetrow(W, 0, acc_avg);
            rt_kprintf("Z+: %.3f %.3f %.3f\n", acc_avg[0], acc_avg[1], acc_avg[2]);
        }

        if (fabs(acc_avg[0]) < ACC_CAL_LIMIT && fabs(acc_avg[1]) < ACC_CAL_LIMIT && fabs(acc_avg[2] - (-GRAVITY)) < ACC_CAL_LIMIT && (acc_bm & 0x02) == 0)
        {
            acc_bm |= 0x02;
            msetrow(W, 1, acc_avg);
            rt_kprintf("Z-: %.3f %.3f %.3f\n", acc_avg[0], acc_avg[1], acc_avg[2]);
        }

        if (fabs(acc_avg[0]) < ACC_CAL_LIMIT && fabs(acc_avg[2]) < ACC_CAL_LIMIT && fabs(acc_avg[1] - GRAVITY) < ACC_CAL_LIMIT && (acc_bm & 0x04) == 0)
        {
            acc_bm |= 0x04;
            msetrow(W, 2, acc_avg);
            rt_kprintf("Y+: %.3f %.3f %.3f\n", acc_avg[0], acc_avg[1], acc_avg[2]);
        }

        if (fabs(acc_avg[0]) < ACC_CAL_LIMIT && fabs(acc_avg[2]) < ACC_CAL_LIMIT && fabs(acc_avg[1] - (-GRAVITY)) < ACC_CAL_LIMIT && (acc_bm & 0x08) == 0)
        {
            acc_bm |= 0x08;
            msetrow(W, 3, acc_avg);
            rt_kprintf("Y-: %.3f %.3f %.3f\n", acc_avg[0], acc_avg[1], acc_avg[2]);
        }

        if (fabs(acc_avg[1]) < ACC_CAL_LIMIT && fabs(acc_avg[2]) < ACC_CAL_LIMIT && fabs(acc_avg[0] - GRAVITY) < ACC_CAL_LIMIT && (acc_bm & 0x10) == 0)
        {
            acc_bm |= 0x10;
            msetrow(W, 4, acc_avg);
            rt_kprintf("X+: %.3f %.3f %.3f\n", acc_avg[0], acc_avg[1], acc_avg[2]);
        }

        if (fabs(acc_avg[1]) < ACC_CAL_LIMIT && fabs(acc_avg[2]) < ACC_CAL_LIMIT && fabs(acc_avg[0] - (-GRAVITY)) < ACC_CAL_LIMIT && (acc_bm & 0x20) == 0)
        {
            acc_bm |= 0x20;
            msetrow(W, 5, acc_avg);
            rt_kprintf("X-: %.3f %.3f %.3f\n", acc_avg[0], acc_avg[1], acc_avg[2]);
        }

        /* we collect all acc data */
        if ((acc_bm & 0x3F) == 0x3F)
        {
            nl_lsq2(W, Y, X, Q);
            mtrans(X, XT);

            mgetcol(XT, 0, tmp);
            msetcol(&imu_mgr->gmis, 0, tmp);
            mgetcol(XT, 1, tmp);
            msetcol(&imu_mgr->gmis, 1, tmp);
            mgetcol(XT, 2, tmp);
            msetcol(&imu_mgr->gmis, 2, tmp);
            mgetcol(XT, 3, imu_mgr->calib.gb);
            v3scale2(imu_mgr->calib.gb, -1);

            acc_bm |= 0x80;
            rt_kprintf("ACC CALIBRATION COMPLETE\n");

            clear_acc_sum();

            if (acc_bm & 0x80)
                break;
        }
        nl_mdelay(2);
    }

    nl_free(W);
    nl_free(Y);
    nl_free(X);
    nl_free(XT);
    nl_free(Q);
    return RT_EOK;
}



void nl_adj_att(nl_t *qin, att_t *att_tgt, nl_t *qout)
{
    nl_t qtgt[4];
    
    att2q(att_tgt, qtgt);
    qconj2(qtgt);
    qmul(qtgt, eskfsvr.ins.q, qout);
    qconj2(qout);
}

void app_cli_cal_cmd_init()
{
    rt_cli_cmd_create("FCAL", FCAL, rt_cli_find(CLI1_NAME));
    rt_cli_cmd_create("FCAL", FCAL, rt_cli_find(CLI2_NAME));
    rt_cli_cmd_create("FCAL", FCAL, rt_cli_find(CLI7_NAME));
}


