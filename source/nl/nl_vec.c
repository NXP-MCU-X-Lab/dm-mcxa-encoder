#include "nl.h"

nl_t *vcreate(size_t num)
{
    nl_t *v = nl_malloc(sizeof(nl_t) * num);
    if(v)
    {
        memset(v, 0, sizeof(nl_t)*num);
    }
    return v;
}



/* A = B */
void vcopy(nl_t *A, nl_t *B, size_t len)
{
    memcpy(A, B, len * sizeof(nl_t));
}

/* A = B */
void v3copy(nl_t *A, nl_t *B)
{
    vcopy(A, B, 3);
}

/* A = val */
void vfill(nl_t *A, nl_t val, size_t len)
{
    size_t i = 0;

    for(i = 0; i < (len & ~3); i += 4) {
        A[i] = val;
        A[i+1] = val;
        A[i+2] = val;
        A[i+3] = val;
    }
    for(; i < len; i++) {
        A[i] = val;
    }
}

/* A = val */
void v3fill(nl_t *A, nl_t val)
{
    vfill(A, val, 3);
}

/* A = A + B */
void vadd2(nl_t *A, nl_t *B, size_t len)
{
    size_t i = 0;
    for(i=0; i<len; i++) A[i] = A[i] + B[i];
}

/* C = A + B */
void vadd(nl_t *C, nl_t *A, nl_t *B, size_t len)
{
    size_t i = 0;
    for(i=0; i<len; i++) C[i] = A[i] + B[i];
}

/* A = A + B */
void v3add2(nl_t *A, nl_t *B)
{
    A[0] = A[0] + B[0];
    A[1] = A[1] + B[1];
    A[2] = A[2] + B[2];
}

/* C = A + B */
void v3add(nl_t *C, nl_t *A, nl_t *B)
{
    C[0] = A[0] + B[0];
    C[1] = A[1] + B[1];
    C[2] = A[2] + B[2];
}

/* A = A - B */
void vsub2(nl_t *A, nl_t *B, size_t len)
{
    size_t i= 0;
    for(i=0; i<len; i++) A[i] = A[i] - B[i];
}

/* C = A - B */
void vsub(nl_t *C, nl_t *A, nl_t *B, size_t len)
{
    size_t i= 0;
    for(i=0; i<len; i++) C[i] = A[i] - B[i];
}

/* A = A - B */
void v3sub2(nl_t *A, nl_t *B)
{
    A[0] = A[0] - B[0];
    A[1] = A[1] - B[1];
    A[2] = A[2] - B[2];
}

/* C = A - B */
void v3sub(nl_t *C, nl_t *A, nl_t *B)
{
    C[0] = A[0] - B[0];
    C[1] = A[1] - B[1];
    C[2] = A[2] - B[2];
}

/* A = B * k */
void vscale(nl_t *A, nl_t *B, nl_t k, size_t len)
{
    size_t i = 0;
    for(i=0; i<len; i++) A[i] = B[i] * k;
}

/* A = A * k */
void vscale2(nl_t *A, nl_t k, size_t len)
{
    size_t i = 0;
    for(i=0; i<len; i++) A[i] *= k;
}

/* A = B * k */
void v3scale(nl_t *A, nl_t *B, nl_t k)
{
    A[0] = B[0] * k;
    A[1] = B[1] * k;
    A[2] = B[2] * k;
}

/* A = A * k */
void v3scale2(nl_t *A, nl_t k)
{
    A[0] = A[0] * k;
    A[1] = A[1] * k;
    A[2] = A[2] * k;
}

/* ret = A . B */
nl_t vdot(nl_t *A, nl_t *B, size_t len)
{
    nl_t result = 0;
    size_t i = 0;
    for(i=0; i<len; i++)
    {
        result += A[i] * B[i];
    }
    return result;
}

/* ret = A . B */
nl_t v3dot(nl_t *A, nl_t *B)
{
    nl_t a0 = A[0], a1 = A[1], a2 = A[2];
    nl_t b0 = B[0], b1 = B[1], b2 = B[2];
    return a0*b0 + a1*b1 + a2*b2;
}

/* C = A x B */
void v3cross(nl_t *A, nl_t *B, nl_t *C)
{
    C[0] = A[1]*B[2] - A[2]*B[1];
    C[1] = A[2]*B[0] - A[0]*B[2];
    C[2] = A[0]*B[1] - A[1]*B[0];
}

nl_t vnorm(nl_t *A, size_t len)
{
    nl_t n = 0;
    size_t i = 0;
    for (i=0, n=0; i<len; i++) n += A[i]*A[i];
    return sqrt(n);
}

nl_t v3norm(nl_t *A)
{
    nl_t x = A[0], y = A[1], z = A[2];
    return sqrtf(x*x + y*y + z*z);
}

/* Normlize a vector A */
void vnormlz(nl_t *A, size_t len)
{
    nl_t x = A[0], y = A[1], z = A[2];
    nl_t norm2 = x*x + y*y + z*z;
    
    nl_t inv_norm = 1.0f / sqrtf(norm2);
    A[0] = x * inv_norm;
    A[1] = y * inv_norm;
    A[2] = z * inv_norm;
}

/* Normlize a vector A */
void v3normlz(nl_t *A)
{
    vnormlz(A, 3);
}


void vconstrain(nl_t *X, nl_t max, nl_t min, size_t len)
{
    size_t i = 0;
    for(i=0; i<len; i++)
    {
        if(X[i] > max) X[i] = max;
        if(X[i] < min) X[i] = min;
    }
}


void v3constrain(nl_t *X, nl_t max, nl_t min)
{
    vconstrain(X, max, min, 3);
}


nl_t sum(nl_t *V, size_t len)
{
    size_t i=0;
    nl_t n = 0;
    for (i=0, n=0; i<len; i++) n += V[i];
    return n;
}

nl_t mean(nl_t *V, size_t len)
{
    return sum(V, len) / len;
}

