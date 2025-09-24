#ifndef __CRTKLIB_H__
#define __CRTKLIB_H__

#include "rtklib.h"



#define EN_TRACE_CEPH       (1)
#define EN_TRACE_CPNTPOS    (1)

#define SQR(x)      ((x)*(x))
#define MAX(x,y)    ((x)>=(y)?(x):(y))
#define MAX_VAR_EPH SQR(300.0)  /* max variance eph to reject satellite (m^2) */


void c_eph2pos(gtime_t time, eph_t *eph, double *rs, double *dts, double *var);
void c_satposs(gtime_t teph, obsd_t* obs, int n, nav_t *nav, double *rs, double *dts, double *var, int *svh);
double c_eph2clk(gtime_t time, eph_t *eph);
int c_rescode(int iter,  obsd_t *obs, int n,  double *rs, double *dts,  double *vare,  int *svh, nav_t *nav,  double *x,  prcopt_t *opt, ssat_t *ssat, double *v, double *H, double *var, double *azel, int *vsat, double *resp, int *ns);
int c_estpos(obsd_t *obs, int n,  double *rs,  double *dts, double *vare,  int *svh,  nav_t *nav, prcopt_t *opt,  ssat_t *ssat, sol_t *sol, double *azel, int *vsat, double *resp, char *msg);   
int c_pntpos(obsd_t *obs, int n, nav_t *nav, prcopt_t *opt, sol_t *sol, double *azel, ssat_t *ssat, char *msg);
                    
                    
                   
#endif