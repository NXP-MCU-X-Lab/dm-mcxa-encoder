#include "nl.h"



/* the default config of ESKF156 */
eskf156_opt_t eskf156_option_example_uncalibrate =
    {
        .P0_att = {2*D2R, 2*D2R, 10*D2R},
        .P0_vel = 1,     
        .P0_pos = 5,
        .P0_gyr_bias = 0.1 * D2R,
        .P0_acc_bias = 0.001 * GRAVITY,    
        .P0_installangle = 2 * D2R,
        .P0_timedelay = 0.1,

        .Pmax_att = 100 * D2R,
        .Pmax_vel = 20,
        .Pmax_pos = 1000,
        .Pmax_gyr_bias = 0.5 * D2R,
        .Pmax_acc_bias = 0.01 * GRAVITY,

        .Pmin_att = 0.02 * D2R,
        .Pmin_vel = 0.005,
        .Pmin_pos = 0.05,
        .Pmin_gyr_bias = 0 * D2R,
        .Pmin_acc_bias = 0 * GRAVITY,

        .Qgyr_wb = 0.05 * D2R,
        .Qacc_wb = 0.01,
        .Qgyr_wbb = (2.0 / 3600) * D2R,
        .Qacc_wbb = 25.0/1e6 * GRAVITY,
        .Q_timedelay = 0.0/3600,

        .Q_installangle= 1/3600 * D2R,
        .R0_gnss_vel = 0.1,
        .R0_gnss_pos = 1,
        .R0_dualgnss = 2,
        .R0_dualgnss_heading = 2 * D2R,
        .R0_zupt = 0.05,
        .R0_nhc = 1.0,
        .R0_gravity = 3.0,
        .R0_od = 0.5,
        .R0_zaru = 0.01 * D2R,

        /* Other options */
        .gnss_delay = 0.03, /* GNSS delay */
};

eskf156_opt_t eskf156_option_example_calibrated =
    {
        .P0_att = {1*D2R, 1*D2R, 2*D2R},
        .P0_vel = 1,     
        .P0_pos = 5,
        .P0_gyr_bias = 0.01 * D2R,
        .P0_acc_bias = 0.001 * GRAVITY,    
        .P0_installangle = 0.02 * D2R,
        .P0_timedelay = 0.1,

        .Pmax_att = 100 * D2R,
        .Pmax_vel = 20,
        .Pmax_pos = 1000,
        .Pmax_gyr_bias = 0.5 * D2R,
        .Pmax_acc_bias = 0.1 * GRAVITY,

        .Pmin_att = 0.02 * D2R,
        .Pmin_vel = 0.005,
        .Pmin_pos = 0.05,
        .Pmin_gyr_bias = 0 * D2R,
        .Pmin_acc_bias = 0 * GRAVITY,

        .Qgyr_wb = 0.05 * D2R,
        .Qacc_wb = 0.01,
        .Qgyr_wbb = (2.0 / 3600) * D2R,
        .Qacc_wbb = 25.0/1e6 * GRAVITY,


        .Q_installangle= 0.1/3600 * D2R,
        .Q_timedelay = 0.0/3600,
        .R0_gnss_vel = 0.1,
        .R0_gnss_pos = 1,
        .R0_dualgnss = 2,
        .R0_dualgnss_heading = 2 * D2R,
        .R0_zupt = 0.05,
        .R0_nhc = 1.0,
        .R0_gravity = 3.0,
        .R0_od = 0.5,
        .R0_zaru = 0.01 * D2R,

        /* Other options */
        .gnss_delay = 0.03, /* GNSS delay */
};
    
/* the default config of ESKF156_ship */
eskf156_opt_t eskf156_option_example_ship =
    {
        .P0_att = {2*D2R, 2*D2R, 2*D2R},
        .P0_vel = 1,
        .P0_pos = 5,
        .P0_gyr_bias = 0.2 * D2R,
        .P0_acc_bias = 0.001 * GRAVITY,
        .P0_installangle = 0.002 * D2R,

        .Pmax_att = 10 * D2R,
        .Pmax_vel = 20,
        .Pmax_pos = 1000,
        .Pmax_gyr_bias = 0.5 * D2R,
        .Pmax_acc_bias = 0.01 * GRAVITY,

        .Pmin_att = 0.02 * D2R,
        .Pmin_vel = 0.005,
        .Pmin_pos = 0.05,
        .Pmin_gyr_bias = 0 * D2R,
        .Pmin_acc_bias = 0 * GRAVITY,

        .Qgyr_wb = 0.05 * D2R,
        .Qacc_wb = 0.01,
        .Qgyr_wbb = (2.0 / 3600) * D2R,
        .Qacc_wbb = 25.0/1e6 * GRAVITY,

        .Q_installangle = 0.0/3600 * D2R,
        .Q_timedelay = 0.0/3600,
        .R0_gnss_vel = 0.1,
        .R0_gnss_pos = 1,
        .R0_dualgnss = 2,
        .R0_dualgnss_heading = 0.5 * D2R,
        .R0_zupt = 0.05,
        .R0_nhc = 1.0,
        .R0_gravity = 3.0,
        .R0_od = 0.5,
        .R0_zaru = 0.05 * D2R,

        /* Other options */
        .gnss_delay = 0.03, /* GNSS delay */
};
/* the default config of ESKF6: [phi, gyr_bias] */
eskfatt_opt_t eskfatt_opt =
{
    .P0_att = {2*D2R, 2*D2R, 0.1*D2R},
    .P0_gyr_bias = 0.1 * D2R,
    
    .Pmax_att = 30 * D2R,
    .Pmin_att = 0 * D2R,
    .Pmax_gyr_bias = (0.3) * D2R,
    .Pmin_gyr_bias = 0 * D2R,
    
    .Qgyr_wb = 0.02 * D2R,
    .Qgyr_wbb = (25.0/3600)*D2R,
    .R0_gravity = 0.04,
    .R0_mag = 2.0,
    .R0_zaru = 0.002,
    .R0_zihr = 2.0 * D2R,
};

/* the default config of ESKF6: [phi, gyr_bias] */
kfmag_opt_t kfmag_opt =
{
    .P0_mag_n = {20, 20, 20},
    .P0_mag_bias = 20,
    
    .Pmax_mag_n = 100,
    .Pmin_mag_n = 0.001,
    .Pmax_mag_bias = 50,
    .Pmin_mag_bias = 0.001,
    
    .Q_mag_n = 0.01,
    .Q_mag_bias = (25.0/3600),
    .R0_mag = 0.5,
};


