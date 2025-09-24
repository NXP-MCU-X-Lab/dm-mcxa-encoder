/**
 * @file nl_gps_time_sync.h
 * @brief GPS time disciplining module with PPS alignment
 */
#ifndef __GPS_TIME_SYNC_H__
#define __GPS_TIME_SYNC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configuration constants
#define PPS_TIMEOUT_MS          (60*1000)     // PPS signal timeout
#define PPS_LOCK_COUNT          4             // Required consecutive valid PPS for lock
#define PPS_INTERVAL_MIN        900           // Min valid PPS interval (ms)
#define PPS_INTERVAL_MAX        1100          // Max valid PPS interval (ms)
#define PPS_STABILIZE_COUNT     2             // Require 2 stable PPS after gap

// Disciplining states
typedef enum {
    DISC_INIT = 0,          // Waiting for initial RMC
    DISC_COARSE,            // RMC received, waiting for PPS lock  
    DISC_LOCKED             // PPS locked, precise timing available
} discipline_state_t;

// UTC time structure
typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t ms;
} gps_utc_time_t;

// Public API
int nl_gps_time_init(void);
int nl_gps_time_input_rmc(const uint8_t *buf, uint32_t len);
int nl_gps_time_input_utc(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint16_t ms);
void nl_gps_time_pps_callback(void);
uint32_t nl_gps_time_get_daily_ms(void);
int nl_gps_time_get_utc(gps_utc_time_t *utc);
int nl_gps_time_is_locked(void);

#ifdef __cplusplus
}
#endif

#endif // __GPS_TIME_SYNC_H__
