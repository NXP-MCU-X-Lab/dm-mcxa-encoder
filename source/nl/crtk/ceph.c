#include "crtklib.h"

#if ( EN_TRACE_CEPH == 1 )
#define CEPH_TRACE	trace
#else
#define CEPH_TRACE(...)
#endif



#define MU_GPS   3.9860050E14     /* gravitational constant         ref [1] */
#define MU_CMP   3.986004418E14   /* earth gravitational constant   ref [9] */
#define OMGE_CMP 7.292115E-5      /* earth angular velocity (rad/s) ref [9] */

#define SIN_5 -0.0871557427476582 /* sin(-5.0 deg) */
#define COS_5  0.9961946980917456 /* cos(-5.0 deg) */

#define RTOL_KEPLER 1E-13         /* relative tolerance for Kepler equation */
#define MAX_ITER_KEPLER 5        /* max number of iteration of Kelpler */

#define MAXDTOE     7200.0              /* max time difference to GPS Toe (s) */



/* variance by ura ephemeris -------------------------------------------------*/
static double var_uraeph(int sys, int ura)
{
    const double ura_value[]=
    {   
        2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
        3072.0,6144.0
    };
    return ura<0||14<ura?SQR(6144.0):SQR(ura_value[ura]);
}


void c_eph2pos(gtime_t time, eph_t *eph, double *rs, double *dts, double *var)               
{   
    double tk,M,E,Ek,sinE,cosE,u,r,i,O,sin2u,cos2u,x,y,sinO,cosO,cosi,mu,omge;
    double xg,yg,zg,sino,coso;
    int n,sys,prn;
    
    if (eph->A<=0.0)
    {
        rs[0] = rs[1] = rs[2] = *dts = *var = 0.0;
        return;
    }
    
    tk = timediff(time, eph->toe);
    
    switch ((sys=satsys(eph->sat,&prn)))
    {
        case SYS_CMP: mu = MU_CMP; omge = OMGE_CMP; break;
        default:      mu = MU_GPS; omge = OMGE;     break;
    }
    
    M = eph->M0+(sqrt(mu/(eph->A*eph->A*eph->A))+eph->deln)*tk;
    
    for (n=0,E=M,Ek=0.0;fabs(E-Ek)>RTOL_KEPLER&&n<MAX_ITER_KEPLER;n++)
    {
        Ek=E; E-=(E-eph->e*sin(E)-M)/(1.0-eph->e*cos(E));
    }

    if (n>=MAX_ITER_KEPLER)
    {
        CEPH_TRACE(2,"eph2pos: kepler iteration overflow sat=%2d\n",eph->sat);
        return;
    }
    sinE = sin(E); cosE = cos(E);
    
    u = atan2(sqrt(1.0-eph->e*eph->e)*sinE,cosE-eph->e)+eph->omg;
    r = eph->A*(1.0-eph->e*cosE);
    i = eph->i0+eph->idot*tk;
    sin2u = sin(2.0*u); cos2u=cos(2.0*u);
    u += eph->cus*sin2u+eph->cuc*cos2u;
    r += eph->crs*sin2u+eph->crc*cos2u;
    i += eph->cis*sin2u+eph->cic*cos2u;
    x = r*cos(u); y = r*sin(u); cosi = cos(i);
    
    /* beidou geo satellite */
    if (sys == SYS_CMP && (prn<=5 || prn>=59))
    { /* ref [9] table 4-1 */
        O=eph->OMG0+eph->OMGd*tk-omge*eph->toes;
        sinO=sin(O); cosO=cos(O);
        xg=x*cosO-y*cosi*sinO;
        yg=x*sinO+y*cosi*cosO;
        zg=y*sin(i);
        sino=sin(omge*tk); coso=cos(omge*tk);
        rs[0]= xg*coso+yg*sino*COS_5+zg*sino*SIN_5;
        rs[1]=-xg*sino+yg*coso*COS_5+zg*coso*SIN_5;
        rs[2]=-yg*SIN_5+zg*COS_5;
    }
    else
    {
        O = eph->OMG0+(eph->OMGd-omge)*tk-omge*eph->toes;
        sinO = sin(O); cosO=cos(O);
        rs[0] = x*cosO-y*cosi*sinO;
        rs[1] = x*sinO+y*cosi*cosO;
        rs[2] = y*sin(i);
    }
    tk = timediff(time, eph->toc);
    *dts = eph->f0+eph->f1*tk+eph->f2*tk*tk;
    
    /* relativity correction */
    *dts-=2.0*sqrt(mu*eph->A)*eph->e*sinE/SQR(CLIGHT);
    
    /* position and clock error variance */
    *var=var_uraeph(sys,eph->sva);
}


/* broadcast ephemeris to satellite clock bias ---------------------------------
* compute satellite clock bias with broadcast ephemeris (gps, galileo, qzss)
* args   : gtime_t time     I   time by satellite clock (gpst)
*          eph_t *eph       I   broadcast ephemeris
* return : satellite clock bias (s) without relativeity correction
* notes  : see ref [1],[7],[8]
*          satellite clock does not include relativity correction and tdg
*-----------------------------------------------------------------------------*/
double c_eph2clk(gtime_t time, eph_t *eph)
{
    double t,ts;
    int i;
    
    CEPH_TRACE(4,"eph2clk : time=%s sat=%2d\n",time_str(time,3), eph->sat);
    
    t = ts = timediff(time, eph->toc);
    
    for (i=0;i<2;i++)
    {
        t = ts-(eph->f0+eph->f1*t+eph->f2*t*t);
    }
    return eph->f0+eph->f1*t+eph->f2*t*t;
}


static eph_t *c_seleph(gtime_t time, int sat, int iode, nav_t *nav)
{
    int i, j=-1, sys, sel=0;
    char s1[64], s2[64];
    
    for (i=0; i<nav->n; i++)
    {
         if (nav->eph[i].sat == sat)
         {
            if ((fabs(timediff(nav->eph[i].toe, time))) > MAXDTOE)
            {
                return NULL;
            }
            else
            {
                return &nav->eph[i];
            }
         }
    }
    return NULL;
}


/* satellite positions and clocks ----------------------------------------------
* compute satellite positions, velocities and clocks
* args   : gtime_t teph     I   time to select ephemeris (gpst)
*          obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          int    ephopt    I   ephemeris option (EPHOPT_???)
*          double *rs       O   satellite positions and velocities (ecef)
*          double *dts      O   satellite clocks
*          double *var      O   sat position and clock error variances (m^2)
*          int    *svh      O   sat health flag (-1:correction not available)
* return : none
* notes  : rs [(0:2)+i*6]= obs[i] sat position {x,y,z} (m)
*          rs [(3:5)+i*6]= obs[i] sat velocity {vx,vy,vz} (m/s)
*          dts[(0:1)+i*2]= obs[i] sat clock {bias,drift} (s|s/s)
*          var[i]        = obs[i] sat position and clock error variance (m^2)
*          svh[i]        = obs[i] sat health flag
*          if no navigation data, set 0 to rs[], dts[], var[] and svh[]
*          satellite position and clock are values at signal transmission time
*          satellite position is referenced to antenna phase center
*          satellite clock does not include code bias correction (tgd or bgd)
*          any pseudorange and broadcast ephemeris are always needed to get
*          signal transmission time
*-----------------------------------------------------------------------------*/
void c_satposs(gtime_t teph, obsd_t* obs, int n, nav_t *nav, double *rs, double *dts, double *var, int *svh)
{
    gtime_t time[MAXOBS] = {{0}};
    double dt, pr;
    int i, j;
    eph_t *eph;
    
    CEPH_TRACE(3,"satposs : teph=%s n=%d\r\n",time_str(teph, 3), n);
    
    /* loop through all obs */
    for (i=0; i<n; i++)
    {
        for (j=0;j<6;j++) rs [j+i*6]=0.0;
        for (j=0;j<2;j++) dts[j+i*2]=0.0;
        var[i]=0.0; svh[i]=0;
        
        /* search any pseudorange */
        pr = obs[i].P[0];

        /* transmission time by satellite clock */
        time[i]=timeadd(obs[i].time, -pr/CLIGHT);
        
        eph = c_seleph(time[i], obs[i].sat, 0, nav);
            
        if(!eph)
        {
            CEPH_TRACE(3,"no broadcast clock %s sat=%2d\n",time_str(time[i],3), obs[i].sat);
            continue;
        }
        
        svh[i] = eph->svh;
        dt = c_eph2clk(time[i], eph);
        time[i] = timeadd(time[i],-dt);
        
        /* satellite position and clock at transmission time */
        c_eph2pos(time[i], eph, rs+i*6, dts+i*2, var+i);
    }
    
    for (i=0; i<n; i++)
    {
        CEPH_TRACE(2,"%s sat=%2d rs=%13.3f %13.3f %13.3f dts=%12.3f var=%7.3f svh=%02X\n", time_str(time[i],6),obs[i].sat,rs[i*6],rs[1+i*6],rs[2+i*6], dts[i*2]*1E9,var[i],svh[i]);    
    }
}



