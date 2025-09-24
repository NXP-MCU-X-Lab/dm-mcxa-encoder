#include "nl.h"

#define NL_DBG_TAG    "nl.mat2"
#define NL_DBG_LVL    NL_DBG_INFO
#include "nl_log.h"

/*
    nl_mat2.c:
    advance matrix algo. 
*/




/* XuShuLiang C yu yan chang yong suan fa ji(3th), to compute eig value */
void jcbj(m_t *A, m_t *V, nl_t eps)
{
    int i,j,p,q,u,w,t,s, n;
    
    n = A->c;
    
    nl_t ff,fm,cn,sn,omega,x,y,d;
    for (i=0; i<=n-1; i++)
      { V->dat[i*n+i]=1.0;
        for (j=0; j<=n-1; j++)
          if (i!=j) V->dat[i*n+j]=0.0;
      }
    ff=0.0;
    for (i=1; i<=n-1; i++)
    for (j=0; j<=i-1; j++)
      { d=A->dat[i*n+j]; ff=ff+d*d; }
    ff=sqrt(2.0*ff);
    loop0:
    ff=ff/(1.0*n);
    loop1:
        for (i=1; i<=n-1; i++)
        for (j=0; j<=i-1; j++)
          { d=fabs(A->dat[i*n+j]);
            if (d>ff)
              { p=i; q=j;
                goto loop;
              }
          }
        if (ff<eps) return;
        goto loop0;
  loop: u=p*n+q; w=p*n+p; t=q*n+p; s=q*n+q;
        x=-A->dat[u]; y=(A->dat[s]-A->dat[w])/2.0;
        omega=x/sqrt(x*x+y*y);
        if (y<0.0) omega=-omega;
        sn=1.0+sqrt(1.0-omega*omega);
        sn=omega/sqrt(2.0*sn);
        cn=sqrt(1.0-sn*sn);
        fm=A->dat[w];
        A->dat[w]=fm*cn*cn+A->dat[s]*sn*sn+A->dat[u]*omega;
        A->dat[s]=fm*sn*sn+A->dat[s]*cn*cn-A->dat[u]*omega;
        A->dat[u]=0.0; A->dat[t]=0.0;
        for (j=0; j<=n-1; j++)
        if ((j!=p)&&(j!=q))
          { u=p*n+j; w=q*n+j;
            fm=A->dat[u];
            A->dat[u]=fm*cn+A->dat[w]*sn;
            A->dat[w]=-fm*sn+A->dat[w]*cn;
          }
        for (i=0; i<=n-1; i++)
          if ((i!=p)&&(i!=q))
            { u=i*n+p; w=i*n+q;
              fm=A->dat[u];
              A->dat[u]=fm*cn+A->dat[w]*sn;
              A->dat[w]=-fm*sn+A->dat[w]*cn;
            }
        for (i=0; i<=n-1; i++)
          { u=i*n+p; w=i*n+q;
            fm=V->dat[u];
            V->dat[u]=fm*cn+V->dat[w]*sn;
            V->dat[w]=-fm*sn+V->dat[w]*cn;
          }
       goto loop1;
}


/* least square estimation -----------------------------------------------------
* least square estimation by solving normal equation (x=(A*A')^-1*A*y)
* args   : double *A        I   design matrix (m x n)
*          double *y        I   (weighted) measurements (m x 1)
*          double *x        O   estmated parameters (n x 1)
*          double *Q        O   esimated parameters covariance matrix (n x n)
* return : status (0:ok,0>:error)
* notes  : for weighted least square, replace A and y by A*w and w*y (w=W^(1/2))
*          matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
int nl_lsq(m_t *A, nl_t *y, nl_t *x, m_t *Q)
{
    int info;
    int m = A->r, n = A->c;
    m_t Y, X;
    Y.r = m;
    Y.c = 1;
    Y.dat = y;
    X.r = n;
    X.c = 1;
    X.dat = x;

    m_t *Ay = mcreate(n, 1);
    if (m < n)
        return -1;
    mmul3("TN", 1.0, A, &Y, 0.0, Ay); /* Ay = A'*y */
    mmul3("TN", 1.0, A, A, 0.0, Q);   /* Q = A'*A */
    info = minv(Q);
    if (!info) mmul(Q, Ay, &X); /* x=Q^-1*Ay */
    nl_free(Ay);
    return info;
}


int nl_lsq2(m_t *A, m_t *Y, m_t *X, m_t *Q)
{
    int info;
    int m = A->r, n = A->c;

    NL_ASSERT(Q->r == n);
    NL_ASSERT(Q->c == n);
    NL_ASSERT(Y->r == m);
    NL_ASSERT(X->r == n);
    NL_ASSERT(X->c == Y->c);

    m_t *Ay = mcreate(n, Y->c);
    if (m < n)
        return -1;
    mmul3("TN", 1.0, A, Y, 0.0, Ay); /* Ay = A'*y */
    mmul3("TN", 1.0, A, A, 0.0, Q);   /* Q = A'*A */
    //mprint(Q, 4);
    info = minv(Q);
    if (!info)  mmul(Q, Ay, X); /* x=Q^-1*Ay */
    nl_free(Ay);
    return info;
}
