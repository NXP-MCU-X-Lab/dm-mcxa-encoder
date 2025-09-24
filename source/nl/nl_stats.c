/**
 * @file nl_stats.c
 * @brief Implementation of statistical calculations
 */

#include "nl_stats.h"
#include <stdlib.h>
#include <string.h>

void nl_stats_basic_update(nl_stats_basic_t *stats, nl_t value)
{
    stats->count++;
    stats->sum += value;
    
    // Welford's online algorithm for mean and variance
    nl_t delta = value - stats->mean;
    stats->mean += delta / stats->count;
    nl_t delta2 = value - stats->mean;
    stats->m2 += delta * delta2;
}

nl_t nl_stats_basic_get_mean(const nl_stats_basic_t *stats)
{
    return (stats->count > 0) ? stats->mean : 0;
}

nl_t nl_stats_basic_get_variance(const nl_stats_basic_t *stats)
{
    return (stats->count > 1) ? (stats->m2 / (stats->count - 1)) : 0;
}

nl_t nl_stats_basic_get_sum(nl_stats_basic_t *stats)
{
    return stats->sum;
}


void nl_stats_basic_reset(nl_stats_basic_t *stats)
{
    stats->mean = 0;
    stats->m2 = 0;
    stats->count = 0;
    stats->sum = 0;
}

/**
 * @brief Create a sliding window statistics calculator
 * @param stats Pointer to statistics structure
 * @param window_size Size of the sliding window
 * @return true if creation successful, false otherwise
 */
nl_stats_window_t *nl_stats_window_create(int window_size)
{
    if (window_size <= 0) {
        return RT_NULL;
    }
        
    nl_stats_window_t *stats = nl_malloc(sizeof(nl_stats_window_t));
    if (!stats) {
        return RT_NULL;
    }
    
    stats->buffer = (nl_t *)nl_malloc(window_size * sizeof(nl_t));
    if (!stats->buffer) {
        nl_free(stats);  // Free previously allocated memory
        return RT_NULL;
    }

    stats->window_size = window_size;
    stats->head = 0;
    stats->count = 0;
    stats->mean = 0;
    stats->m2 = 0;

    memset(stats->buffer, 0, window_size * sizeof(nl_t));
    return stats;
}


/**
 * @brief Update sliding window statistics with a new value using Welford's algorithm
 * @param stats Pointer to statistics structure
 * @param value New value to add to the window
 */
void nl_stats_window_update(nl_stats_window_t *stats, nl_t value)
{
    // If window is full, remove oldest value
    if (stats->count == stats->window_size) {
        nl_t old_value = stats->buffer[stats->head];
        
        // Update mean for removal
        nl_t new_mean = (stats->count * stats->mean - old_value) / (stats->count - 1);
        
        // Update M2 for removal
        stats->m2 = stats->m2 - (old_value - stats->mean) * (old_value - new_mean);
        stats->mean = new_mean;
        
        // Now add new value using standard Welford update
        nl_t delta = value - stats->mean;
        stats->mean += delta / stats->count;
        stats->m2 += delta * (value - stats->mean);
    } else {
        // Window not full yet, just add new value using Welford
        stats->count++;
        nl_t delta = value - stats->mean;
        stats->mean += delta / stats->count;
        stats->m2 += delta * (value - stats->mean);
    }

    // Store new value in circular buffer
    stats->buffer[stats->head] = value;
    stats->head = (stats->head + 1) % stats->window_size;
}

/**
 * @brief Get current mean value of the window
 * @param stats Pointer to statistics structure
 * @return Mean value, 0 if window is empty
 */
nl_t nl_stats_window_get_mean(const nl_stats_window_t *stats)
{
    return (stats->count > 0) ? stats->mean : 0;
}

/**
 * @brief Get current variance of the window
 * @param stats Pointer to statistics structure
 * @return Variance value, 0 if window has less than 2 samples
 */
nl_t nl_stats_window_get_variance(const nl_stats_window_t *stats)
{
    return (stats->count > 1) ? (stats->m2 / (stats->count - 1)) : 0;
}

/**
 * @brief Reset the sliding window statistics
 * @param stats Pointer to statistics structure
 */
void nl_stats_window_reset(nl_stats_window_t *stats)
{
    memset(stats->buffer, 0, stats->window_size * sizeof(nl_t));
    stats->head = 0;
    stats->count = 0;
    stats->mean = 0;
    stats->m2 = 0;
}

void nl_stats_window_free(nl_stats_window_t *stats)
{
    if (stats) {
        if (stats->buffer) {
            nl_free(stats->buffer);
        }
        nl_free(stats);
    }
}



