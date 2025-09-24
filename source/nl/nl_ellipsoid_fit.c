#include "nl.h"

/**
 * @brief Calculate the residuals for ellipsoid fitting using raw data points
 * @param data Pointer to the input data array (size*3 elements)
 * @param size Number of data points
 * @param C Pointer to the calibration matrix (3x3)
 * @param B Pointer to the bias vector (3x1)
 * @param n Estimated norm (field strength)
 * @return Calculated residual
 */
nl_t nl_ellipsoid_fit_get_residuals(nl_t *data, uint32_t size, m_t *C, nl_t *B, nl_t n)
{
    int i;
    nl_t residual = 0, point[3] = {0}, tmp[3] = {0}, res = 0;
    
    for(i = 0; i < size; i++)
    {
        // Get the current data point
        point[0] = data[i*3];
        point[1] = data[i*3+1];
        point[2] = data[i*3+2];
        
        // Remove bias
        v3sub2(point, B);
        
        // Apply calibration matrix
        mvmul(C, point, tmp);
        
        // Calculate residual for this point
        res = POW2(tmp[0]) + POW2(tmp[1]) + POW2(tmp[2]) - POW2(n);
        residual += POW2(res);
    }
    
    return 1 / (2*POW2(n)) * sqrt(residual / size);
}

/**
 * @brief Perform 4-point sphere fitting (3 bias + 1 norm)
 * @param D Pointer to the input data matrix (3xN)
 * @param C Pointer to the output calibration matrix (3x3), will be set to identity
 * @param B Pointer to the output bias vector (3x1, hard iron)
 * @param norm Pointer to the estimated norm (1x1, expected field strength)
 * @param fit_err Pointer to the estimated fit error
 * @return 0 on success
 */
int nl_sphere_fit(m_t *D, m_t *C, nl_t *B, nl_t *norm, nl_t *fit_err)
{
    int  i;
    
    m_t *A = mcreate(D->r, 4);
    m_t *Y = mcreate(D->r, 1);
    m_t *X = mcreate(4, 1);
    m_t *Q = mcreate(4, 4);
    
    meye(C);
    
    for(i=0; i<D->r; i++)
    {
        MELEMENT(A, i, 0) = MELEMENT(D, i, 0);
        MELEMENT(A, i, 1) = MELEMENT(D, i, 1);
        MELEMENT(A, i, 2) = MELEMENT(D, i, 2);
        MELEMENT(A, i, 3) = 1;
        
        MELEMENT(Y, i, 0) = POW2(MELEMENT(D, i, 0)) + POW2(MELEMENT(D, i, 1)) + POW2(MELEMENT(D, i, 2));
    }
    
    nl_lsq2(A, Y, X, Q);
    
    /* extract center and field strength */
    B[0] = X->dat[0] / 2;
    B[1] = X->dat[1] / 2;
    B[2] = X->dat[2] / 2;
    *norm = sqrt(X->dat[3] + POW2(B[0]) +  POW2(B[1]) +  POW2(B[2]));
    
    *fit_err = nl_ellipsoid_fit_get_residuals(D->dat, D->r, C, B, *norm);
    
    nl_free(A);
    nl_free(Y);
    nl_free(X);
    nl_free(Q);
    return 0;
}

/**
 * @brief Perform ellipsoid fitting
 * @param D Pointer to the input data matrix (3xN)
 * @param C Pointer to the output calibration matrix (3x3)
 * @param B Pointer to the output bias vector (3x1, hard iron)
 * @param norm Pointer to the estimated norm (1x1, expected field strength)
 * @param fit_err Pointer to the estimated fit error
 * @return 0 on success
 */
int nl_ellipsoid_fit(m_t *D, m_t *C, nl_t *B, nl_t *norm, nl_t *fit_err)
{
    int n = D->r, i, min_idx;
    nl_t beta[7], min_eig_value = 10000, dA;
    
    m_t *A = mcreate(n, 7);
    m_t *At = mcreate(7, n);
    m_t *AtA = mcreate(7, 7);
    m_t *V = mcreate(AtA->r, AtA->c);
    
    /* build design matrix */
    for(i=0; i<n; i++)
    {
        MELEMENT(A, i, 0) = POW2(MELEMENT(D, i, 0));
        MELEMENT(A, i, 1) = POW2(MELEMENT(D, i, 1));
        MELEMENT(A, i, 2) = POW2(MELEMENT(D, i, 2));
        MELEMENT(A, i, 3) = MELEMENT(D, i, 0);
        MELEMENT(A, i, 4) = MELEMENT(D, i, 1);
        MELEMENT(A, i, 5) = MELEMENT(D, i, 2);
        MELEMENT(A, i, 6) = 1;
    }

    mtrans(A, At);
    mmul(At, A, AtA);

    /* eigenvalue decomposition */
    nl_t eps = 0.00001;
    jcbj(AtA, V, eps); 

    /* find smallest eigenvalue */
    for(i=0; i<AtA->r; i++)
    {
        if(MELEMENT(AtA, i, i) < min_eig_value)
        {
            min_eig_value = MELEMENT(AtA, i, i);
            min_idx = i;
        }
    }
    
    mgetcol(V, min_idx, beta);
    
    mfill(C, 0);
    msetdiag(C, beta);
    
    dA = det(C);
    if(dA < 0)
    {
        vscale2(beta, -1, 7);
        mscale2(C, -1);
        dA = -dA;
    }
    
    /* calculate hard iron offset */
    B[0] = -0.5 * beta[3] / beta[0];
    B[1] = -0.5 * beta[4] / beta[1];
    B[2] = -0.5 * beta[5] / beta[2];
    
    /* calculate geomagnetic field strength */
    *norm = MELEMENT(C, 0, 0)*POW2(B[0]) + MELEMENT(C, 1, 1)*POW2(B[1]) + MELEMENT(C, 2, 2)*POW2(B[2]) - beta[6];
    *norm = sqrt(fabs(*norm));

    /* normalize C */
    nl_t det3root = pow(dA, 1.0 / 3);
    nl_t det6root = sqrt(det3root);
    
    MELEMENT(C, 0, 0) /= det3root;
    MELEMENT(C, 1, 1) /= det3root;
    MELEMENT(C, 2, 2) /= det3root;
    MELEMENT(C, 0, 0) = sqrt(MELEMENT(C, 0, 0));
    MELEMENT(C, 1, 1) = sqrt(MELEMENT(C, 1, 1));
    MELEMENT(C, 2, 2) = sqrt(MELEMENT(C, 2, 2));
    *norm = *norm / det6root; 
    
    *fit_err = nl_ellipsoid_fit_get_residuals(D->dat, D->r, C, B, *norm);
    
    nl_free(A);
    nl_free(At);
    nl_free(AtA);
    nl_free(V);
    return 0;
}
