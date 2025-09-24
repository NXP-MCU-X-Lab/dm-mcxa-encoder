#include "nl.h"

#define NL_DBG_TAG    "nl.mat"
#define NL_DBG_LVL    NL_DBG_INFO
#include "nl_log.h"


m_t *mcreate(size_t r, size_t c)
{
    m_t *A = nl_malloc(sizeof(m_t) + r*c*sizeof(nl_t));
    if(!A)
    {
        NL_ASSERT(A != NULL);
        return NULL;
    }
    
    A->dat = (nl_t*)((uint32_t)A + sizeof(m_t));
    A->r = r;
    A->c = c;
    A->s = r*c;
    
    memset(A->dat, 0, A->s*sizeof(nl_t));

    return A;
}

m_t *mcreate2(size_t r, size_t c, nl_t *p)
{
    m_t *A = nl_malloc(sizeof(m_t));
    if(!A)
    {
        NL_ASSERT(A != NULL);
        return NULL;
    }
    
    A->dat = p;
    A->r = r;
    A->c = c;
    A->s = r*c;
    
    return A;
}

void minit(m_t *A, size_t r, size_t c, nl_t *p)
{
    A->dat = p;
    A->r = r;
    A->c = c;
    A->s = r*c;
}



/* A = val */
void mfill(m_t *A, nl_t val)
{
    int i;
    for(i=0; i<A->s; i++)
    {
        A->dat[i] = val;
    }
}

/* diag(A) = val */
void mfilldiag(m_t *A, nl_t val)
{
    if (A->r != A->c)
    {
        return;
    }

    for (int i = 0; i < A->c; i++)
    {
        MELEMENT(A, i, i) = val;
    }
}

void msetdiag(m_t *A, nl_t *V)
{
    if (A->r != A->c)
    {
        return;
    }

    for (int i = 0; i < A->c; i++)
    {
        MELEMENT(A, i, i) = V[i];
    }
}

void nl_skew(m_t *m, nl_t *v)
{
    MELEMENT(m, 0, 0) =  0;
    MELEMENT(m, 0, 1) = -v[2];
    MELEMENT(m, 0, 2) =  v[1];
    
    MELEMENT(m, 1, 0) =  v[2];
    MELEMENT(m, 1, 1) =  0;
    MELEMENT(m, 1, 2) = -v[0];
    
    MELEMENT(m, 2, 0) = -v[1];
    MELEMENT(m, 2, 1) =  v[0];
    MELEMENT(m, 2, 2) =  0;
}


/* A = B */
void mcopy(m_t *A, m_t *B)
{
    for(int i=0; i<A->s; i++)
    {
        A->dat[i] = B->dat[i];
    }
}

/* A(r,c) = B (Block copy of matrix) */
void mbcopy(m_t *A, m_t *m, size_t start_r, size_t start_c)
{
    for(int r=0; r<m->r; r++)
    {
        for(int c=0; c<m->c; c++)
        {
            MELEMENT(A, start_r+r, start_c+c) = MELEMENT(m, r, c);
        }
    }
}

/* C = A + B */
void madd(m_t *A, m_t *B, m_t *C)
{
    int j;

    for(j=0; j<A->s; ++j)
    {
        C->dat[j] = A->dat[j] + B->dat[j];
    }
}

/* A = A + B */
void madd2(m_t *A, m_t *B)
{
    int i;

    for(i=0; i<A->s; ++i)
    {
        A->dat[i] = A->dat[i] + B->dat[i];
    }
}

/* C = A - B */
void msub(m_t *C, m_t *A, m_t *B)
{
    int i;

    for(i=0; i<A->s; ++i)
    {
        C->dat[i] = A->dat[i] - B->dat[i];
    }
}

/* A = A - B */
void msub2(m_t *A, m_t *B)
{
    int i;

    for(i=0; i<A->s; ++i)
    {
        A->dat[i] = A->dat[i] - B->dat[i];
    }
}

/* B = A * k */
void mscale(m_t *A, m_t *B, nl_t k)
{
    int j;
    for (j=0; j<A->s; ++j)
    {
        B->dat[j] = A->dat[j] * k;
    }
}

/* A = A * k */
void mscale2(m_t *A, nl_t k)
{
    int j;
    for (j=0; j<A->s; ++j)
    {
        A->dat[j] *= k;
    }
}

/* A = A + diag(d) */
void madddiag(m_t *A, nl_t *d)
{
    int j;
    for (j=0; j<A->r; ++j)
    {
        MELEMENT(A, j, j) += d[j];
    }
}

/* A = A + alpha * diag(d) */
void madddiag2(m_t *A, nl_t alpha, nl_t *d)
{
    int j;
    for (j=0; j<A->r; ++j)
    {
        MELEMENT(A, j, j) += alpha * d[j];
    }
}

void meye(m_t *A)
{
    mfill(A, 0);
    mfilldiag(A, 1);
}


/* set one row in a matrix */
void msetrow(m_t *A, int r, nl_t* V)
{
    int i;
    for(i=0; i<A->c; i++)
    {
        MELEMENT(A, r, i) = V[i];
    }
}

void mgetrow(m_t *A, int r, nl_t* V)
{
    int i;
    for(i=0; i<A->c; i++)
    {
        V[i] = MELEMENT(A, r, i);
    }
}

void mgetcol(m_t *A, int c, nl_t *V)
{
    int i;
    for(i=0; i<A->r; i++)
    {
        V[i] = MELEMENT(A, i, c);
    }
}

void msetcol(m_t *A, int c, nl_t* V)
{
    int i;
    for(i=0; i<A->r; i++)
    {
        MELEMENT(A, i, c) = V[i];
    }
}



/* A = A + I */
void maddeye(m_t *A)
{
    int j;
    for (j=0; j<A->r; ++j)
    {
        MELEMENT(A, j, j) += 1;
    }
}

/* s = A * v */
void mvmul(m_t *A, nl_t *v, nl_t *s)
{
    int i, j;

    for(i=0; i<A->r; i++)
    {
        s[i] = 0;
        for(j=0; j<A->c; j++)
        {
            s[i] += v[j] * MELEMENT(A, i, j);
        }
    }
}

/* C = A * B */
void mmul(m_t *A, m_t *B, m_t *C)
{
    mmul3("NN", 1, A, B, 0, C);
}

/* C = alpha*A*B + beta*C */
void mmul3(const char *tr, nl_t alpha, m_t *matA, m_t *matB, nl_t beta, m_t *matC)
{
     int n = matC->r;
     int m = (tr[0]=='N' ? matA->c:matA->r);
     volatile int k = matC->c;
     nl_t *A = matA->dat;
     nl_t *B = matB->dat;
     nl_t *C = matC->dat;
    
    nl_t d;
    int i,j,x,f=tr[0]=='N'?(tr[1]=='N'?1:2):(tr[1]=='N'?3:4);
    
    for (i=0;i<n;i++) for (j=0;j<k;j++) {
        d=0.0; 
        switch (f) {
            case 1: for (x=0;x<m;x++) d+=A[i*m+x]*B[x*k+j]; break;
            case 2: for (x=0;x<m;x++) d+=A[i*m+x]*B[j*m+x]; break;
            case 3: for (x=0;x<m;x++) d+=A[x*n+i]*B[x*k+j]; break;
            case 4: for (x=0;x<m;x++) d+=A[x*n+i]*B[j*m+x]; break;
        }
        if (beta==0) C[i*k+j]=alpha*d; else C[i*k+j]=alpha*d+beta*C[i*k+j];
    }
}

nl_t mtrace(m_t *A)
{
    int i;
    nl_t trace = 0;
    
    NL_ASSERT(A->c == A->r);

    for(i=0; i<A->r; ++i)
    {
        trace += MELEMENT(A, i, i);
    }
    return trace;
}

/* AT = A' */
void mtrans(m_t *A, m_t *AT)
{
    int i, j;
    NL_ASSERT((A->c == AT->r) && (A->r == AT->c));
    
    for(i=0; i<A->r; ++i)
    {
        for(j=0; j<A->c; ++j)
        {
            MELEMENT(AT, j, i) = MELEMENT(A, i, j);
        }
    }
}

nl_t *nl_mat(int n, int m)
{
    nl_t *p = nl_malloc(sizeof(nl_t)*n*m);
    NL_ASSERT(p != NULL);
    return p;
}

int *nl_imat(int n, int m)
{
    int *p = nl_malloc(sizeof(int)*n*m);
    NL_ASSERT(p != NULL);
    return p;
}

nl_t det(m_t *A)
{
    m_t *AC = mcreate(A->r, A->c);
    mcopy(AC, A);
    
    int n = AC->r;
    nl_t *a = AC->dat;
    int i, j, k, is, js, l, u, v;
    nl_t f, det, q, d;
    f = 1.0;
    det = 1.0;
    for (k = 0; k <= n - 2; k++)
    {
        q = 0.0;
        for (i = k; i <= n - 1; i++)
            for (j = k; j <= n - 1; j++)
            {
                l = i * n + j;
                d = fabs(a[l]);
                if (d > q)
                {
                    q = d;
                    is = i;
                    js = j;
                }
            }
        if (q + 1.0 == 1.0)
        {
            det = 0.0;
            return (det);
        }
        if (is != k)
        {
            f = -f;
            for (j = k; j <= n - 1; j++)
            {
                u = k * n + j;
                v = is * n + j;
                d = a[u];
                a[u] = a[v];
                a[v] = d;
            }
        }
        if (js != k)
        {
            f = -f;
            for (i = k; i <= n - 1; i++)
            {
                u = i * n + js;
                v = i * n + k;
                d = a[u];
                a[u] = a[v];
                a[v] = d;
            }
        }
        l = k * n + k;
        det = det * a[l];
        for (i = k + 1; i <= n - 1; i++)
        {
            d = a[i * n + k] / a[l];
            for (j = k + 1; j <= n - 1; j++)
            {
                u = i * n + j;
                a[u] = a[u] - d * a[k * n + j];
            }
        }
    }
    det = f * det * a[n * n - 1];
    
    nl_free(AC);
    return (det);
}

void nl_matcpy(nl_t *A, const nl_t *B, int n, int m)
{
    memcpy(A, B,sizeof(nl_t)*n*m);
}

/* LU decomposition ----------------------------------------------------------*/
static int nl_ludcmp(nl_t *A, volatile int n, int *indx, nl_t *d)
{
    
    nl_t big, s, tmp, *vv = vcreate(n);
    int i, imax = 0, j, k;

    *d = 1.0;
    for (i = 0; i < n; i++)
    {
        big = 0.0;
        for (j = 0; j < n; j++)
            if ((tmp = fabs(A[i + j * n])) > big)
                big = tmp;
        if (big > 0.0)
            vv[i] = 1.0 / big;
        else
        {
            nl_free(vv);
            return -1;
        }
    }
    for (j = 0; j < n; j++)
    {
        for (i = 0; i < j; i++)
        {
            s = A[i + j * n];
            for (k = 0; k < i; k++)
                s -= A[i + k * n] * A[k + j * n];
            A[i + j * n] = s;
        }
        big = 0.0;
        for (i = j; i < n; i++)
        {
            s = A[i + j * n];
            for (k = 0; k < j; k++)
                s -= A[i + k * n] * A[k + j * n];
            A[i + j * n] = s;
            if ((tmp = vv[i] * fabs(s)) >= big)
            {
                big = tmp;
                imax = i;
            }
        }
        if (j != imax)
        {
            for (k = 0; k < n; k++)
            {
                tmp = A[imax + k * n];
                A[imax + k * n] = A[j + k * n];
                A[j + k * n] = tmp;
            }
            *d = -(*d);
            vv[imax] = vv[j];
        }
        indx[j] = imax;
        if (A[j + j * n] == 0.0)
        {
            nl_free(vv);
            return -1;
        }
        if (j != n - 1)
        {
            tmp = 1.0 / A[j + j * n];
            for (i = j + 1; i < n; i++)
                A[i + j * n] *= tmp;
        }
    }
    nl_free(vv);
    return 0;
}

static void nl_lubksb(const nl_t *A, volatile int n, const int *indx, nl_t *b)
{
    nl_t s;
    int i, ii = -1, ip, j;

    for (i = 0; i < n; i++)
    {
        ip = indx[i];
        s = b[ip];
        b[ip] = b[i];
        if (ii >= 0)
            for (j = ii; j < i; j++)
                s -= A[i + j * n] * b[j];
        else if (s)
            ii = i;
        b[i] = s;
    }
    for (i = n - 1; i >= 0; i--)
    {
        s = b[i];
        for (j = i + 1; j < n; j++)
            s -= A[i + j * n] * b[j];
        b[i] = s / A[i + i * n];
    }
}

int minv(m_t *A)
{
    nl_t d, *B;
    int i, j, *indx, n = A->r;
    nl_t *D = A->dat;

    indx = nl_imat(n, 1);
    B = nl_mat(n, n);
    nl_matcpy(B, D, n, n);
    if (nl_ludcmp(B, n, indx, &d))
    {
        nl_free(indx);
        nl_free(B);
        return -1;
    }
    for (j = 0; j < n; j++)
    {
        for (i = 0; i < n; i++)
            D[i + j * n] = 0.0;
        D[j + j * n] = 1;
        nl_lubksb(B, n, indx, D + j * n);
    }
    nl_free(indx);
    nl_free(B);
    return 0;
}


