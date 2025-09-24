#include "crtklib.h"


#if ( EN_TRACE_CPNTPOS == 1 )
#define CPNTPOS_TRACE	printf
#else
#define CPNTPOS_TRACE(...)
#endif


#define NX          (4+2)
#define MAXITR      (10)            /* max number of iteration for point pos */
#define ERR_CBIAS   (0.3)           /* code bias error Std (m) */
#define MIN_EL      (5.0*D2R)       /* min elevation for measurement error (rad) */


/* get group delay parameter (m) ---------------------------------------------*/
static double gettgd(int sat, nav_t *nav, int type)
{
    int i,sys = satsys(sat,NULL);
    
    for (i=0;i<nav->n;i++)
    {
        if (nav->eph[i].sat==sat) break;
    }
    return (i>=nav->n)?0.0:nav->eph[i].tgd[type]*CLIGHT;
}


/* pseudorange measurement error variance ------------------------------------*/
static double varerr( prcopt_t *opt, double el, double snr_rover, int sys)
{
    double fact=1.0,varr;

    switch (sys) {
        case SYS_GPS: fact *= EFACT_GPS; break;
        case SYS_CMP: fact *= EFACT_CMP; break;
        default:      fact *= EFACT_GPS; break;
    }
    if (el<MIN_EL) el=MIN_EL;
    if (opt->weightmode==WEIGHTOPT_ELEVATION) {
        /* var = R^2 * (a^2 + b^2 / sin(el)) */
        varr=SQR(opt->eratio[0])*(SQR(opt->err[1])+SQR(opt->err[2])/sin(el));
    } else { /* WEIGHTOPT_SNR */
        /* var = R^2 * (a^2 * (10^(0.1*(snr_max-snr_rover)) */
        varr=SQR(opt->eratio[0])*SQR(opt->err[1])*pow(10,0.1*MAX(opt->err[5]-snr_rover,0));
    } 

    if (opt->ionoopt==IONOOPT_IFLC) varr*=SQR(3.0); /* iono-free */
    return SQR(fact)*varr;
}


/* iono-free or "pseudo iono-free" pseudorange with code bias correction -----*/
/*
obsd_t   *obs      I   observation data
nav_t    *nav      I   navigation data
double   *azel     I   对于当前定位值，每一颗观测卫星的 {方位角、高度角}
int      iter      I   迭代次数
prcopt_t *opt      I   processing options
double   *vare     O   伪距测量的码偏移误差
返回类型：
double             O   最终能参与定位解算的伪距值
*/
static double prange(obsd_t *obs,  nav_t *nav, const prcopt_t *opt, double *var)
{
    double P1,P2,gamma,b1,b2;
    int sat, sys;
    
    sat = obs->sat;
    sys = satsys(sat,NULL);
    P1 = obs->P[0];
    P2 = obs->P[1];
    *var = 0.0;
    
    if (P1 == 0.0 || (opt->ionoopt==IONOOPT_IFLC&&P2==0.0)) return 0.0;
    
    *var = SQR(ERR_CBIAS);
        
    if (sys == SYS_GPS)  /* L1 */
    {
        b1 = gettgd(sat,nav,0); /* TGD (m) */
        return P1-b1;
    }

    if (sys == SYS_CMP)  /* B1I/B1Cp/B1Cd */
    {
        if      (obs->code[0] == CODE_L2I) b1 = gettgd(sat, nav,0); /* TGD_B1I */
        else if (obs->code[0] == CODE_L1P) b1 = gettgd(sat, nav,2); /* TGD_B1Cp */
        else b1 = gettgd(sat,nav,2) + gettgd(sat, nav,4); /* TGD_B1Cp+ISC_B1Cd */
        return P1 - b1;
    }
    return P1;
}

/* test SNR mask -------------------------------------------------------------*/
static int snrmask(obsd_t *obs, const double *azel, const prcopt_t *opt)
{
    if (testsnr(0,0,azel[1],obs->SNR[0]*SNR_UNIT,&opt->snrmask))
    {
        return 0;
    }
    if (opt->ionoopt==IONOOPT_IFLC) {
        if (testsnr(0,1,azel[1],obs->SNR[1]*SNR_UNIT,&opt->snrmask)) return 0;
    }
    return 1;
}


/* validate solution ---------------------------------------------------------*/
static int valsol(double *azel, int *vsat, int n, prcopt_t *opt, double *v, int nv, int nx, char *msg)           
{
    double azels[MAXOBS*2], dop[4], vv;
    int i,ns;
    
    CPNTPOS_TRACE(2,"valsol  : n=%d nv=%d\n",n,nv);
    
    /* Chi-square validation of residuals */
    vv=dot(v,v,nv);
    printf("vv:%f\r\n", vv);
    if (nv>nx&&vv>chisqr[nv-nx-1]*CHISQR)
    {
        CPNTPOS_TRACE(1, "Warning: large chi-square error nv=%d vv=%.1f cs=%.1f\r\n",nv,vv,chisqr[nv-nx-1]);
        /* return 0; */ /* threshold too strict for all use cases, report error but continue on */
    }
    /* large GDOP check */
    for (i=ns=0;i<n;i++)
    {
        if (!vsat[i]) continue;
        azels[  ns*2]=azel[  i*2];
        azels[1+ns*2]=azel[1+i*2];
        ns++;
    }
    dops(ns,azels,opt->elmin,dop);
    if (dop[0]<=0.0||dop[0]>opt->maxgdop)
    {
        CPNTPOS_TRACE(1, "gdop error nv=%d gdop=%.1f",nv,dop[0]);
        return 0;
    }
    return 1;
}

/* estimate receiver position ------------------------------------------------*/
/*
通过伪距实现绝对定位，计算出接收机的位置和钟差，顺带返回实现定位后每颗卫星的{方位角、仰角}、定位时有效性、定位后伪距残差。
函数参数，13个：
obsd_t   *obs      I   observation data
int      n         I   number of observation data
double   *rs       I   satellite positions and velocities，长度为6*n，{x,y,z,vx,vy,vz}(ecef)(m,m/s)
double   *dts      I   satellite clocks，长度为2*n， {bias,drift} (s|s/s)
double   *vare     I   sat position and clock error variances (m^2)
int      *svh      I   sat health flag (-1:correction not available)
nav_t    *nav      I   navigation data
prcopt_t *opt      I   processing options
sol_t    *sol      IO  solution
double   *azel     IO  azimuth/elevation angle (rad)
int      *vsat     IO  表征卫星在定位时是否有效
double   *resp     IO  定位后伪距残差 (P-(r+c*dtr-c*dts+I+T))
char     *msg      O   error message for error exit
返回类型:
int                O     (1:ok,0:error)
*/
int c_estpos(obsd_t *obs, int n,  double *rs,  double *dts, double *vare,  int *svh,  nav_t *nav, prcopt_t *opt, ssat_t *ssat, sol_t *sol, double *azel, int *vsat, double *resp, char *msg)             
{
    double x[NX]={0}, dx[NX], Q[NX*NX], *v, *H, *var, sig;
    int i,j,k,info,stat,nv,ns;
    
    CPNTPOS_TRACE(3,"estpos  : n=%d\n",n);
    
    v=mat(n,1); H=mat(NX,n); var=mat(n,1);
    
    for (i=0;i<3;i++) x[i]=sol->rr[i];
    
    for (i=0;i<MAXITR;i++)
    {
        /* pseudorange residuals (m) */
        nv = c_rescode(i,obs,n,rs,dts,vare,svh,nav,x,opt,ssat,v,H,var,azel,vsat,resp, &ns);
                       
        if (nv < NX)
        {
            CPNTPOS_TRACE(1,"lack of valid sats ns=%d", nv);
            break;
        }
        
        /* weighted by Std */
        for (j=0;j<nv;j++)
        {
            sig=sqrt(var[j]);
            v[j]/=sig;
            for (k=0;k<NX;k++) H[k+j*NX]/=sig;
        }
            
        //matfprint(H, NX, nv, 2, 2, NULL);
            
        /* least square estimation */
        if ((info=lsq(H,v,NX,nv,dx,Q))) {
            CPNTPOS_TRACE(1, "lsq error info=%d",info);
            break;
        }
        for (j=0;j<NX;j++) {
            x[j]+=dx[j];
        }
        
        if (norm(dx,NX)<1E-4)
        {
                sol->type=0;
                sol->time=timeadd(obs[0].time,-x[3]/CLIGHT);
                sol->dtr[0]=x[3]/CLIGHT; /* receiver clock bias (s) */
                sol->dtr[1]=x[4]/CLIGHT; /* GAL-GPS time offset (s) */
                sol->dtr[2]=x[5]/CLIGHT; /* BDS-GPS time offset (s) */
                for (j=0;j<6;j++) sol->rr[j]=j<3?x[j]:0.0;
                for (j=0;j<3;j++) sol->qr[j]=(float)Q[j+j*NX];
                sol->qr[3]=(float)Q[1];    /* cov xy */
                sol->qr[4]=(float)Q[2+NX]; /* cov yz */
                sol->qr[5]=(float)Q[2];    /* cov zx */
                sol->ns=(uint8_t)ns;
                sol->age=sol->ratio=0.0;
                
                /* validate solution */
                if ((stat = valsol(azel, vsat, n, opt, v, nv, NX, msg)))
                {
                    sol->stat=opt->sateph==EPHOPT_SBAS?SOLQ_SBAS:SOLQ_SINGLE;
                }
                free(v); free(H); free(var);
                return stat;
        }
    }
    if (i>=MAXITR) CPNTPOS_TRACE(1,"iteration divergent i=%d",i);
    
    free(v); free(H); free(var);
    return 0;
}

/*
使用伪距残差判决法对计算得到的定位结果进行接收机自主正直性检测（RAIM），每次舍弃一颗卫星测量值，用剩余的值组成一组进行定位运算，选择定位后伪距残差最小的一组作为最终结果。这样如果只有一个异常观测值的话，这个错误可以被排除掉；有两个或以上错误则排除不了。注意这里只会在对定位结果有贡献的卫星数据进行检测。

函数参数，13个：
obsd_t   *obs      I   观测数据
int      n         I   观测数据的数量
double   *rs       I   卫星位置和速度，长度为6*n，{x,y,z,vx,vy,vz}(ecef)(m,m/s)
double   *dts      I   卫星钟差，长度为2*n， {bias,drift} (s|s/s)
double   *vare     I   卫星位置和钟差的协方差 (m^2)
int      *svh      I   卫星健康标志 (-1:correction not available)
nav_t    *nav      I   导航数据
prcopt_t *opt      I   处理过程选项
sol_t    *sol      IO  solution
double   *azel     IO  方位角和俯仰角 (rad)
int      *vsat     IO  卫星在定位时是否有效
double   *resp     IO  定位后伪距残差 (P-(r+c*dtr-c*dts+I+T))
char     *msg      O   错误消息
返回类型:
int                O   (1:ok,0:error)
源码中有很多关于 i、j、k的循环。其中，i表示最外面的大循环，每次将将第 i颗卫星舍弃不用，这是通过 if (j==i) continue实现的；j表示剩余使用的卫星的循环，每次进行相应数据的赋值；k表示参与定位的卫星的循环，与 j一起使用。
*/
static int c_raim_fde( obsd_t *obs, int n,  double *rs, double *dts,  double *vare, int *svh, nav_t *nav,  prcopt_t *opt, ssat_t *ssat, sol_t *sol, double *azel, int *vsat, double *resp, char *msg)          
{
    obsd_t *obs_e;
    sol_t sol_e = {{0}};
    char tstr[32],name[16],msg_e[128];
    double *rs_e,*dts_e,*vare_e,*azel_e,*resp_e,rms_e,rms=100.0;
    int i,j,k,nvsat,stat=0,*svh_e,*vsat_e,sat=0;
    
    CPNTPOS_TRACE(1,"raim_fde: %s n=%2d\n",time_str(obs[0].time,0),n);
    
    if (!(obs_e=(obsd_t *)malloc(sizeof(obsd_t)*n))) return 0;
    rs_e = mat(6,n); dts_e = mat(2,n); vare_e=mat(1,n); azel_e=zeros(2,n);
    svh_e=imat(1,n); vsat_e=imat(1,n); resp_e=mat(1,n); 
    
    for (i=0;i<n;i++) {
        
        /* satellite exclution */
        for (j=k=0;j<n;j++) {
            if (j==i) continue;
            obs_e[k]=obs[j];
            matcpy(rs_e +6*k,rs +6*j,6,1);
            matcpy(dts_e+2*k,dts+2*j,2,1);
            vare_e[k]=vare[j];
            svh_e[k++]=svh[j];
        }
        /* estimate receiver position without a satellite */
        if (!c_estpos(obs_e, n-1, rs_e, dts_e, vare_e, svh_e, nav, opt, ssat, &sol_e, azel_e, vsat_e,resp_e,msg_e))
        {
            CPNTPOS_TRACE(2,"raim_fde: exsat=%2d (%s)\n",obs[i].sat,msg);
            continue;
        }
        for (j=nvsat=0,rms_e=0.0;j<n-1;j++) {
            if (!vsat_e[j]) continue;
            rms_e+=SQR(resp_e[j]);
            nvsat++;
        }
        if (nvsat<5) {
            CPNTPOS_TRACE(1,"raim_fde: exsat=%2d lack of satellites nvsat=%2d\n",
                  obs[i].sat,nvsat);
            continue;
        }
        rms_e=sqrt(rms_e/nvsat);
        
        CPNTPOS_TRACE(1,"raim_fde: exsat=%2d rms=%8.3f\n",obs[i].sat,rms_e);
        
        if (rms_e>rms) continue;
        
        /* save result */
        for (j=k=0;j<n;j++) {
            if (j==i) continue;
            matcpy(azel+2*j,azel_e+2*k,2,1);
            vsat[j]=vsat_e[k];
            resp[j]=resp_e[k++];
        }
        stat=1;

        *sol=sol_e;
        sat=obs[i].sat;
        rms=rms_e;
        vsat[i]=0;
        strcpy(msg,msg_e);
    }
    if (stat) {
        time2str(obs[0].time,tstr,2); satno2id(sat,name);
        CPNTPOS_TRACE(1,"%s: %s excluded by raim\n",tstr+11,name);
    }
    free(obs_e);
    free(rs_e ); free(dts_e ); free(vare_e); free(azel_e);
    free(svh_e); free(vsat_e); free(resp_e);
    return stat;
}

/* single-point positioning ----------------------------------------------------
* compute receiver position, velocity, clock bias by single-point positioning
* with pseudorange and doppler observables
* args   : obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          prcopt_t *opt    I   processing options
*          sol_t  *sol      IO  solution
*          double *azel     IO  azimuth/elevation angle (rad) (NULL: no output)
*          ssat_t *ssat     IO  satellite status              (NULL: no output)
*          char   *msg      O   error message for error exit
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
int c_pntpos(obsd_t *obs, int n, nav_t *nav, prcopt_t *opt, sol_t *sol, double *azel, ssat_t *ssat, char *msg)        
{
    prcopt_t opt_ = *opt;
    double *rs, *dts, *var, *azel_, *resp;
    int i, stat, vsat[MAXOBS]={0}, svh[MAXOBS];
    
    CPNTPOS_TRACE(3,"pntpos  : tobs=%s n=%d\n",time_str(obs[0].time,3),n);
    
    sol->stat = SOLQ_NONE;
    
    if (n <= 0)
    {
        strcpy(msg,"no observation data");
        return 0;
    }
    
    sol->time = obs[0].time;
    msg[0] = '\0';
    
    rs=mat(6,n); dts=mat(2,n); var=mat(1,n); azel_=zeros(2,n); resp=mat(1,n);
    
    if (ssat) {
        for (i=0;i<MAXSAT;i++) {
            ssat[i].snr_rover[0]=0;
            ssat[i].snr_base[0] =0;
        }
        for (i=0;i<n;i++)
            ssat[obs[i].sat-1].snr_rover[0]=obs[i].SNR[0];
    }
    
    if (opt_.mode != PMODE_SINGLE) /* for precise positioning */
    { 
        opt_.ionoopt = IONOOPT_BRDC;
        opt_.tropopt = TROPOPT_SAAS;
    }
    /* satellite positons, velocities and clocks */
    c_satposs(sol->time, obs, n, nav, rs, dts, var, svh);
    
    /* estimate receiver position with pseudorange */
    stat = c_estpos(obs, n, rs, dts, var, svh, nav, &opt_,ssat, sol, azel_, vsat, resp, msg);
    
    /* RAIM FDE */
    if (!stat&&n>=6&&opt->posopt[4])
    {
        stat = c_raim_fde(obs,n,rs,dts,var,svh,nav,&opt_,ssat,sol,azel_,vsat,resp,msg);
    }
    
    /* estimate receiver velocity with Doppler */
    if (stat)
    {
       // estvel(obs,n,rs,dts,nav,&opt_,sol,azel_,vsat);
    }
    
    if (azel)
    {
        for (i=0;i<n*2;i++) azel[i]=azel_[i];
    }
    if (ssat)
    {
        for (i=0;i<MAXSAT;i++) {
            ssat[i].vs=0;
            ssat[i].azel[0]=ssat[i].azel[1]=0.0;
            ssat[i].resp[0]=ssat[i].resc[0]=0.0;
        }
        for (i=0;i<n;i++) {
            ssat[obs[i].sat-1].azel[0]=azel_[  i*2];
            ssat[obs[i].sat-1].azel[1]=azel_[1+i*2];
            if (!vsat[i]) continue;
            ssat[obs[i].sat-1].vs=1;
            ssat[obs[i].sat-1].resp[0]=resp[i];
        }
    }
    
    free(rs); free(dts); free(var); free(azel_); free(resp);
    return stat;
}

/*
计算在当前接收机位置和钟差值的情况下，定位方程的右端部分 v(nv\*1)、几何矩阵 H(NX*nv)、此时所得的伪距残余的方差 var、所有观测卫星的 azel{方位角、仰角}、定位时有效性 vsat、定位后伪距残差 resp、参与定位的卫星个数 ns和方程个数 nv。

函数参数，17个
int      iter      I    迭代次数
obsd_t   *obs      I    observation data
int      n         I    number of observation data
double   *rs       I   satellite positions and velocities，长度为6*n，{x,y,z,vx,vy,vz}(ecef)(m,m/s)
double   *dts      I   satellite clocks，长度为2*n， {bias,drift} (s|s/s)
double   *vare     I   sat position and clock error variances (m^2)
int      *svh      I   sat health flag (-1:correction not available)
nav_t    *nav      I   navigation data
double   *x        I   本次迭代开始之前的定位值
prcopt_t *opt      I   processing options
double   *v        O   定位方程的右端部分，伪距残余
double   *H        O   定位方程中的几何矩阵
double   *var      O   参与定位的伪距残余方差
double   *azel     O   对于当前定位值，每一颗观测卫星的 {方位角、高度角}
int      *vsat     O   每一颗观测卫星在当前定位时是否有效
double   *resp     O   每一颗观测卫星的伪距残余， (P-(r+c*dtr-c*dts+I+T))
int      *ns       O   参与定位的卫星的个数
返回类型：
int                O   定位方程组的方程个数
*/

int c_rescode(int iter,  obsd_t *obs, int n,  double *rs,
                    double *dts,  double *vare,  int *svh,
                    nav_t *nav,  double *x,  prcopt_t *opt,
                    ssat_t *ssat, double *v, double *H, double *var, 
                   double *azel, int *vsat, double *resp, int *ns)
{
    gtime_t time;
    double r,freq,dion=0.0,dtrp=0.0,vmeas,vion=0.0,vtrp=0.0,rr[3],pos[3],dtr,e[3],P;
    double snr_rover = (ssat) ? SNR_UNIT * ssat->snr_rover[0] : opt->err[5];
    int i,j,nv=0,sat,sys,mask[NX-3]={0};
    
    CPNTPOS_TRACE(3,"resprng : n=%d\n",n);
    
    for (i=0;i<3;i++) rr[i]=x[i];
    dtr = x[3];
    
    ecef2pos(rr, pos);
    
    for (i=*ns=0; i<n; i++)
    {
        vsat[i]=0; azel[i*2] = azel[1+i*2] = resp[i] = 0.0;
        time = obs[i].time;
        sat = obs[i].sat;
        if (!(sys=satsys(sat,NULL))) continue;
        
        /* excluded satellite? */
        if (satexclude(sat,vare[i],svh[i],opt)) continue;
    
        /* geometric distance and elevation mask*/
        if ((r = geodist(rs+i*6, rr, e))<=0.0) continue;
        if (satazel(pos, e, azel+i*2)<opt->elmin) continue;
        
        if (iter > 0)
        {
            /* test SNR mask */
            if (!snrmask(obs+i,azel+i*2,opt)) continue;
        
            /* ionospheric correction */
            if (!ionocorr(time,nav,sat,pos,azel+i*2,opt->ionoopt,&dion,&vion)) {
                continue;
            }
            if ((freq=sat2freq(sat,obs[i].code[0],nav))==0.0) continue;
            dion*=SQR(FREQL1/freq);
            vion*=SQR(FREQL1/freq);
        
            /* tropospheric correction */
            if (!tropcorr(time,nav,pos,azel+i*2,opt->tropopt,&dtrp,&vtrp)) {
                continue;
            }
        }
        /* psendorange with code bias correction */
        if ((P = prange(obs+i, nav, opt, &vmeas)) == 0.0) continue;
        
        /* pseudorange residual */
        v[nv] = P - (r+dtr-CLIGHT*dts[i*2]+dion+dtrp);
        
        /* design matrix */
        for (j=0; j<NX; j++) /* colomn */
        {
            H[j+nv*NX]=j<3?-e[j]:(j==3 ? 1.0:0.0);
        }
        
        /* time system offset and receiver bias correction */
        if      (sys==SYS_GAL) {v[nv]-=x[4]; H[4+nv*NX]=1.0; mask[1]=1;}
        else if (sys==SYS_CMP) {v[nv]-=x[5]; H[5+nv*NX]=1.0; mask[2]=1;}
        else mask[0]=1;
        
        vsat[i]=1; resp[i]=v[nv]; (*ns)++;
        
        /* variance of pseudorange error */
        var[nv++] = varerr(opt, azel[1+i*2], snr_rover, sys)+vare[i]+vmeas+vion+vtrp;
        
        CPNTPOS_TRACE(2,"c_rescode: sat=%2d azel=%5.0f %4.0f res=%7.3f sig=%5.3f\n",obs[i].sat, azel[i*2]*R2D,azel[1+i*2]*R2D,resp[i],sqrt(var[nv-1]));
    }
    
    /* constraint to avoid rank-deficient */
    for (i=0;i<NX-3;i++) {
        if (mask[i]) continue;
        v[nv]=0.0;
        for (j=0;j<NX;j++) H[j+nv*NX]=j==i+3?1.0:0.0;
        var[nv++]=0.01;
    }
    //printf("nv:%d ns:%d\r\n", nv, *ns);
    return nv;
}
                       