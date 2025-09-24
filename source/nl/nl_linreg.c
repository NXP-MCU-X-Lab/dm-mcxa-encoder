#include "nl.h"

/**
 * @brief Create a new linear regression instance
 * @return Pointer to the newly created linreg_t structure, NULL if allocation fails
 */
linreg_t *linreg_create(void)
{
    linreg_t *lr = nl_malloc(sizeof(linreg_t));
    if(lr)
    {
        linreg_clear(lr);
    }
    return lr;
}

/**
 * @brief Clear/Initialize all fields in the linear regression structure
 * @param lr Pointer to the linear regression structure
 */
void linreg_clear(linreg_t *lr)
{
    lr->sum_x = 0;    // Sum of x values
    lr->sum_y = 0;    // Sum of y values
    lr->sum_xy = 0;   // Sum of x*y products
    lr->sum_x2 = 0;   // Sum of x squared values
    lr->p0 = 0;       // Slope coefficient
    lr->p1 = 0;       // Y-intercept
    lr->n = 0;        // Number of data points
}

/**
 * @brief Add a new data point to the linear regression calculation
 * @param lr Pointer to the linear regression structure
 * @param x X coordinate of the data point
 * @param y Y coordinate of the data point
 */
void linreg_add(linreg_t *lr, nl_t x, nl_t y)
{
    lr->sum_x += x;
    lr->sum_y += y;
    lr->sum_xy += x*y;
    lr->sum_x2 += POW2(x);  // POW2(x) calculates x^2
    lr->n++;
}

/**
 * @brief Calculate the linear regression coefficients
 * @param lr Pointer to the linear regression structure
 * @note Fits data to the equation y = p0*x + p1
 *       where p0 is the slope and p1 is the y-intercept
 *       Uses the least squares method to calculate coefficients
 */
void linreg_fit(linreg_t *lr)
{
    // Calculate slope (p0) using the least squares formula
    lr->p0 = (lr->n * lr->sum_xy - lr->sum_x * lr->sum_y) / (lr->n * lr->sum_x2 - POW2(lr->sum_x));
    // Calculate y-intercept (p1)
    lr->p1 = (lr->sum_y - lr->p0 * lr->sum_x) / lr->n;
}
