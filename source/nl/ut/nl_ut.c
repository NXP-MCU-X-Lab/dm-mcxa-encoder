
#include "nl.h"
#include "nl_ut.h"


#define NL_DBG_TAG    "nl.ut"
#define NL_DBG_LVL    NL_DBG_LOG
#include "nl_log.h"


#ifndef RT_NULL
#define RT_NULL 0
#endif

#ifndef RT_EOK
#define RT_EOK 0
#endif

#ifndef RT_ERROR
#define RT_ERROR 1
#endif


int nl_ut_mmul(int argc, char** argv);
int nl_ut_mbcopy(int argc, char** argv);
int nl_ut_minv(int argc, char** argv);
int nl_ut_det(int argc, char** argv);
int nl_ut_ntrip(int argc, char** argv);
int nl_ut_gnss_spp(int argc, char **argv);
int nl_ut_rtcm(int argc, char** argv);
int nl_ut_lsq(int argc, char** argv);
    



int nl_ut_qtest(int argc, char** argv)
{
    NL_LOG_D("test quaternions mathlib\r\n");
    /* do test here */

    nl_t rv[3] = {1, 2, 3};
    nl_t q[4] = {0.182574185835055, 0.365148371670111, 0.547722557505166, 0.730296743340221};
    nl_t dq[4] = {0};
    nl_t rq[4] = {0};

    rv2q(rv, dq);

    vprint(q, 4, 6);
    vprint(dq, 4, 6);

    qmul(q, dq, rq);
    qmul2(q, dq);

    vprint(q, 4, 6);
    vprint(rq, 4, 6);

    nl_t r[3];
    qmulv(q, rv, r);
    vprint(r, 3, 6);
    return RT_EOK;
}

int nl_ut_q2att(int argc, char** argv)
{
    NL_LOG_D("test quaternion to euler angles conversion\r\n");

    nl_t q[4] = {0.9313, -0.0476, 0.3049, -0.1935};
    att_t att;
    q2att(q, &att);
    
    NL_LOG_D("R:%.3f, P:%.3f, Y:%.3f\n", att.roll*R2D, att.pitch*R2D, att.yaw*R2D);

    m_t *dcm = mcreate(3, 3);
    
    q2dcm(q, dcm->dat);
    mprint(dcm, 3);
    
    /* Ref.
        EUL: R:34.168, P:-11.926, Y:-19.798
        
        DCM:    
        0.739    0.331    0.586
       -0.389    0.921   -0.029
       -0.549   -0.207    0.810
    */
    return RT_EOK;
}

int nl_ut_align(int argc, char** argv)
{
    NL_LOG_D("test init align\r\n");

    ins_t ia;
    nl_t a[3]= {1,2,3};
    nl_t y=60*D2R;

    ins_align(&ia, a, y);

    vprint(ia.q, 4, 6);
    return RT_EOK;
}



int nl_ut_kf(int argc, char** argv)
{
    int i;
    kf_state_t kf_s;
    kf_meas_t kf_m;
    kf_state_create(&kf_s, 3);
    kf_meas_create(&kf_m, &kf_s, 2);
    
    nl_t lambda;
    
    nl_t xx[]= {0.814724,0.913376,0.278498};
    nl_t FF[]= {0.964889,0.957167,0.141886,
                0.157613,0.485376,0.421761,
                0.970593,0.800280,0.915736
               };
    nl_t PP[]= {0.792207,0.000000,0.000000,
                0.000000,0.035712,0.000000,
                0.000000,0.000000,0.678735
               };
    nl_t QQ[]= {0.959492426392903,0.849129305868777,0.757740130578333};
    nl_t RR[]= {0.655477890177557,0.706046088019609};
    nl_t HH[]= {0.031833,0.046171,0.823458,
                0.276923,0.097132,0.694829
               };

    nl_t zz[]= {0.655740699156587,0.933993247757551};
    kf_s.X->dat = xx;
    kf_s.F->dat = FF;
    kf_s.P->dat = PP;
    vcopy(kf_s.Q, QQ, ARRAY_SIZE(QQ));
    kf_m.Z->dat = zz;
    vcopy(kf_m.R, RR, ARRAY_SIZE(RR));
    kf_m.H->dat = HH;

    for(i=0; i<100; i++)
    {
        kf_state_update(&kf_s, 0.1);
        kf_m.Z->dat[0]= i;
        kf_m.Z->dat[1]= i*2;
        
        lambda = kf_meas_get_chi_lambda(&kf_m);
        kf_meas_update(&kf_m);
    }
    mprint(kf_s.X, 6);
    mprint(kf_s.P, 6);

    nl_t gt_x[] = {134.792413162237,           98.887071219519,          230.503108099948};
    nl_t gt_P_data[] = {
        0.277771393450245,         0.073047005872154,         0.187242522716693,
        0.073047005872154,         0.139273151055612,         0.107386123961765,
        0.187242522716693,         0.107386123961765,         0.331220672505998
    };

    nl_check_vec(gt_x, M2V(kf_s.X), 3, 0.01);
    nl_check_vec(M2V(kf_s.P), gt_P_data, kf_s.P->s, 0.01);
    

    NL_LOG_D("lambda:%f\r\n", lambda);
    
    return RT_EOK;
}



int nl_ut_inskf(int argc, char** argv)
{
    ins_t ins = {0};
    ins_init(&ins);
    
    nl_t lambda_vel, lambda_pos; /* see: doi:10.19306/j.cnki.2095-8110.2023.04.007 */

    ins.q[0] = 0.533215448243828;
    ins.q[1] = 0.592817248117098;
    ins.q[2] = 0.0831095662269988;
    ins.q[3] = 0.597780725760344;
    
    ins.p[0] = 0;
    ins.p[1] = 0;
    ins.p[2] = 0;

    vfill(ins.v, 0, 3);
    vfill(ins.wb, 0, 3);
    vfill(ins.gb, 0, 3);

    nl_t acc[] = {0.546882,0.957507,9.964889};
    nl_t gyro[] = {0.814724,0.905792,0.126987};

    kf_meas_t mes_pos = {0}, mes_vel = {0}, g= {0}, zupt= {0}, dual_gnss= {0}, nhc= {0}, od= {0}, zihr={0};

    static const eskf156_opt_t opt =
    {
        .P0_att = {2*D2R, 2*D2R, 180*D2R},
        .P0_vel = 0.5,
        .P0_pos = 5,
        .P0_gyr_bias = 50.0/3600.0*D2R,
        .P0_acc_bias = 10e-3*GRAVITY,

        .Pmax_att = 200*D2R,
        .Pmax_vel = 10,
        .Pmax_pos = 100,
        .Pmax_gyr_bias = 50.0/3600.0*D2R,
        .Pmax_acc_bias = 10e-3*GRAVITY,

        .Pmin_att = 0.01*D2R,
        .Pmin_vel = 0.0005,
        .Pmin_pos = 0.001,
        .Pmin_gyr_bias = 0,
        .Pmin_acc_bias = 0,

        .Qgyr_wb =  1.0/60.0*D2R,
        .Qacc_wb =  2.0/60.0,
        .Qgyr_wbb = 0,
        .Qacc_wbb = 0,

        .R0_gnss_vel = 0.1,
        .R0_gnss_pos = 1,

        .R0_nhc = 2.0,
        .R0_zupt = 0.1,
        .R0_gravity = 0.8,
        .R0_od = 0.1,
    };

    lc_eskf_t lc_eskf;
    
    lc_eskf_create(&lc_eskf, &opt);
    lc_eskf_create_meas_pos(&mes_pos, &lc_eskf, opt.R0_gnss_pos);
    lc_eskf_create_meas_vel(&mes_vel, &lc_eskf, opt.R0_gnss_vel);
    

    nl_t gnss_vel[]= {0.1,0.2,0.3};
    nl_t gnss_pos[]= {1,2,3};
    nl_t lever[]= {0.0,0.0,0.0};
    for(int i=0; i<100; i++)
    {
        ins_update(&ins, gyro, acc, 0.01);

        //    if(!(i % 10))
        {
            lc_eskf15_state_model(&lc_eskf.s, &ins, 0.01);
            kf_state_update(&lc_eskf.s, 0.01);
        }


        lc_eskf_meas_set_pos(&mes_pos, &ins, gnss_pos, 0, lever);
        lambda_pos = kf_meas_get_chi_lambda(&mes_pos);
        kf_meas_update(&mes_pos);

        lc_eskf_meas_set_vel(&mes_vel, &ins, gyro, gnss_vel, 0, lever);
        lambda_vel = kf_meas_get_chi_lambda(&mes_vel);
        kf_meas_update(&mes_vel);
        
        //lc_eskf_meas_set_gravity(&g, &ins, acc);
        //kf_meas_update(&s, &g);

        //eskf156_zuptZ_set(&zupt, &ins);
        //kf_meas_update(&s, &zupt);

        //lc_eskf_meas_set_nhc(&nhc, &ins);
        //kf_meas_update(&s, &nhc);

        //lc_eskf_meas_set_od(&od, &ins, 2.0);
        //kf_meas_update(&s, &od);

        lc_eskf_feedback(&lc_eskf, &ins, ESKF_FB_A_XYZ | ESKF_FB_V_XYZ | ESKF_FB_P_XYZ, 1.0);
    }

    NL_LOG_D("q:\r\n");
    vprint(ins.q, 4, 9);
    
    NL_LOG_D("v:\r\n");
    vprint(ins.v, 3, 9);
    
    NL_LOG_D("p:\r\n");
   // vprint(ins.p, 3, 9);
    
    NL_LOG_D("chi_vel:%f, chi_pos:%f\r\n", lambda_vel, lambda_pos);


    /* matlab result: need to remov kf_P_constrain(s); */
    nl_t gt_p[] = {1.065806417516672, 2.117807836992684, 3.145586099040181};
    nl_t gt_v[] = {-0.220469213557383, 0.384513898853226, 0.122751387998753};
    nl_t gt_q[] = {-0.157889003475663,-0.117517919390821,0.234321107563844,0.952026375555619};
    nl_t gt_lambda_vel = 17.918028754742728;
    nl_t gt_lambda_pos = 0.039803662541660;
    
    
//    nl_check_vec(gt_p, ins.p, 3, 0.01);
    nl_check_vec(gt_v, ins.v, 3, 0.01);
    nl_check_vec(gt_q, ins.q, 3, 0.01);
    return RT_EOK;
}



int nl_ut_ins(int argc, char** argv)
{
    nl_t a[3], g[3];
    a[0]=0.1;
    a[1]=-0.2;
    a[2]=10;
    g[0]=0.1;
    g[1]=-0.2;
    g[2]=-0.3;

    ins_t ins = {0};

    ins_init(&ins);

    int cnt=0;

    while(cnt<1000)
    {
        ins_update(&ins, g, a, 0.01);
        cnt++;
    }

    NL_LOG_D("q:\r\n");
    vprint(ins.q, 4, 9);
    
    NL_LOG_D("v:\r\n");
    vprint(ins.v, 3, 9);
    
    NL_LOG_D("p:\r\n");
  //  vprint(ins.p, 3, 9);

    /* matlab result */
    nl_t gt_p[] = {-246.448067612688, 68.8508086451482, -126.381561634328};
    nl_t gt_v[] = {-51.2650131401607, 35.4240370743677, -40.3710290962988};
    nl_t gt_q[] = {-0.295551127492979, 0.255321860045268, -0.510643720090532, -0.765965580135789};
         
//    nl_check_vec(gt_p, ins.p, 3, 0.01);
    nl_check_vec(gt_v, ins.v, 3, 0.01);
    nl_check_vec(gt_q, ins.q, 3, 0.01);
    
    return RT_EOK;
}


//typedef struct  
//{
//    char        *name;
//    int         (*cmd)(int argc, char * argv[]);
//}cmd_t;



///* AT command list */
//cmd_t CDM_TABLE[] = 
//{
//    {"qtest",      nl_ut_qtest},
//    {"align",      nl_ut_align},
//    {"mmul",       nl_ut_mmul},
//    {"mbcopy",     nl_ut_mbcopy},
//    {"kf",         nl_ut_kf},
//    {"eskf",      nl_ut_inskf},
//    {"minv",      nl_ut_minv},
//    {"det",      nl_ut_det},
//    {"ins",      nl_ut_ins},
////    {"rtcm",      nl_ut_rtcm},
//    {"lsq",      nl_ut_lsq},
////    {"span",      nl_ut_span},
////    {"nmea",      nl_ut_nmea},
////    {"malloc",      nl_ut_malloc},
////    {"ubx",      nl_ut_ubx},
////    {"hinpuc",      nl_ut_hipnuc},
////    {"airoha",      nl_ut_airoha},
////    {"ntrip",      nl_ut_ntrip},
////    {"gnss_spp",      nl_ut_gnss_spp},
////    {"gnss_rtk",      nl_ut_gnss_rtk},
//};


//int nl_test(int argc, char** argv)
//{

//    int i;
//    
//    if(argc < 2)
//    {
//        NL_LOG_D("invalid input\r\n");
//        return -1;
//    }
//    
//    for(i=0; i<ARRAY_SIZE(CDM_TABLE); i++)
//    {
//        if(strstr(CDM_TABLE[i].name, argv[1]))
//        {
//            return CDM_TABLE[i].cmd(argc, argv);
//        }
//    }
//    
//    NL_LOG_D("command not found\r\n");
//    return -1;
//}



