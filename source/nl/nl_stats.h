/**
 * @file nl_stats.h
 * @brief Statistical calculations for real-time data processing
 * 
 * Provides two implementations:
 * 1. Basic stats: No buffer, suitable for batch processing with manual reset
 * 2. Window stats: With circular buffer, suitable for sliding window statistics
 */

#ifndef NL_STATS_H
#define NL_STATS_H

#include <stdbool.h>
#include "nl.h"

/* Basic statistics without buffer */
typedef struct {
    nl_t mean;     // Current mean
    nl_t m2;       // Sum of squared differences from mean
    nl_t sum;
    int count;     // Number of samples processed
} nl_stats_basic_t;

/**
 * @brief Structure for sliding window statistics using Welford's algorithm
 */
typedef struct {
    nl_t *buffer;     // Circular buffer for samples
    int window_size;  // Size of the sliding window
    int head;         // Head pointer for circular buffer
    int count;        // Current number of samples
    nl_t mean;        // Running mean
    nl_t m2;          // Sum of squared differences from current mean
} nl_stats_window_t;

/* Basic statistics functions */
void nl_stats_basic_update(nl_stats_basic_t *stats, nl_t value);
nl_t nl_stats_basic_get_mean(const nl_stats_basic_t *stats);
nl_t nl_stats_basic_get_variance(const nl_stats_basic_t *stats);
nl_t nl_stats_basic_get_sum(nl_stats_basic_t *stats);
void nl_stats_basic_reset(nl_stats_basic_t *stats);

/* Sliding window statistics functions */
nl_stats_window_t *nl_stats_window_create(int window_size);
void nl_stats_window_update(nl_stats_window_t *stats, nl_t value);
nl_t nl_stats_window_get_mean(const nl_stats_window_t *stats);
nl_t nl_stats_window_get_variance(const nl_stats_window_t *stats);
void nl_stats_window_reset(nl_stats_window_t *stats);
void nl_stats_window_free(nl_stats_window_t *stats);

#endif // NL_STATS_H


