/**
 * @file nl_gps_time_sync.c
 * @brief GPS time disciplining implementation with proper PPS alignment
 */
#include "nl_gps_time_sync.h"
#include "nl.h"
#include <string.h>
#include <stdlib.h>

#define NL_DBG_TAG    "nl.gps_time_sync"
#define NL_DBG_LVL    NL_DBG_ERROR
#include "nl_log.h"

// GPS disciplining context
static struct {
    // PPS-based precise timing (primary time source when locked)
    uint32_t pps_local_ms;      // Local timestamp of last PPS edge
    uint32_t pps_utc_second;    // UTC second corresponding to last PPS
    
    // RMC-based coarse timing (for initial sync and validation)
    uint32_t rmc_local_ms;      // Local timestamp when RMC was received
    uint32_t rmc_utc_ms;        // UTC time from RMC (daily milliseconds)
    uint32_t last_rmc_update;   // Last RMC coarse sync time
    
    // Date information (from RMC only)
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t date_valid;
    
    // Disciplining state
    discipline_state_t state;
    uint32_t state_enter_time;
    
    // PPS tracking
    uint32_t pps_count;
    uint32_t valid_pps_count;
    uint32_t last_pps_time;
    
    uint8_t pps_unstable_count;     // Count unstable PPS after recovery
    
    // Clock discipline
//    int16_t clock_offset_ms;
    
} g_disc = {0};

// Static functions
static void update_discipline_state(void);
static uint8_t validate_nmea_checksum(const uint8_t *data, uint32_t len);
static int parse_rmc_time_date(const uint8_t *rmc_data, uint32_t len);

// NMEA message buffer
static uint8_t g_nmea_buf[256];
static uint32_t g_nmea_len = 0;

/**
 * @brief Initialize GPS time disciplining module
 */
int nl_gps_time_init(void)
{
    memset(&g_disc, 0, sizeof(g_disc));
    g_disc.state = DISC_INIT;
    g_disc.state_enter_time = nl_get_sys_ms();
    
    NL_LOG_I("GPS time disciplining initialized");
    return 0;
}

/**
 * @brief Validate NMEA checksum
 */
static uint8_t validate_nmea_checksum(const uint8_t *data, uint32_t len)
{
    if (len < 5 || data[0] != '$') return 0;
    
    // Find checksum position
    uint32_t star_pos = 0;
    for (uint32_t i = 1; i < len - 2; i++) {
        if (data[i] == '*') {
            star_pos = i;
            break;
        }
    }
    
    if (star_pos == 0 || star_pos + 3 > len) return 0;
    
    // Calculate checksum
    uint8_t calc_checksum = 0;
    for (uint32_t i = 1; i < star_pos; i++) {
        calc_checksum ^= data[i];
    }
    
    // Parse received checksum
    uint8_t recv_checksum = 0;
    char hex1 = data[star_pos + 1];
    char hex2 = data[star_pos + 2];
    
    if (hex1 >= '0' && hex1 <= '9') recv_checksum += (hex1 - '0') << 4;
    else if (hex1 >= 'A' && hex1 <= 'F') recv_checksum += (hex1 - 'A' + 10) << 4;
    else return 0;
    
    if (hex2 >= '0' && hex2 <= '9') recv_checksum += (hex2 - '0');
    else if (hex2 >= 'A' && hex2 <= 'F') recv_checksum += (hex2 - 'A' + 10);
    else return 0;
    
    return (calc_checksum == recv_checksum);
}

/**
 * @brief Parse RMC time and date fields
 */
static int parse_rmc_time_date(const uint8_t *rmc_data, uint32_t len)
{
    // RMC format: $GNRMC,time,status,lat,lat_dir,lon,lon_dir,speed,course,date,mag_var,checksum
    uint8_t field = 0;
    uint32_t field_start = 0;
    uint32_t time_ms = 0;
    uint16_t year = 0;
    uint8_t month = 0, day = 0;
    uint8_t time_valid = 0, date_valid = 0;
    
    for (uint32_t i = 0; i < len; i++) {
        if (rmc_data[i] == ',' || rmc_data[i] == '*') {
            uint32_t field_len = i - field_start;
            
            if (field == 1 && field_len >= 6) {  // Time field: HHMMSS.sss
                uint8_t *p = (uint8_t*)&rmc_data[field_start];
                if (p[2] < '6' && p[4] < '6') {  // Basic validation
                    uint8_t h = (p[0]-'0')*10 + (p[1]-'0');
                    uint8_t m = (p[2]-'0')*10 + (p[3]-'0');
                    uint8_t s = (p[4]-'0')*10 + (p[5]-'0');
                    uint16_t ms = 0;
                    
                    if (field_len >= 10 && p[6] == '.') {
                        ms = (p[7]-'0')*100 + (p[8]-'0')*10 + (p[9]-'0');
                    }
                    
                    if (h < 24 && m < 60 && s < 60) {
                        time_ms = h*3600000 + m*60000 + s*1000 + ms;
                        time_valid = 1;
                    }
                }
            }
            else if (field == 2 && field_len >= 1) {  // Status field
                if (rmc_data[field_start] != 'A') {
                    return 0;  // Invalid fix
                }
            }
            else if (field == 9 && field_len >= 6) {  // Date field: DDMMYY
                uint8_t *p = (uint8_t*)&rmc_data[field_start];
                uint8_t d = (p[0]-'0')*10 + (p[1]-'0');
                uint8_t mo = (p[2]-'0')*10 + (p[3]-'0');
                uint8_t y = (p[4]-'0')*10 + (p[5]-'0');
                
                if (d > 0 && d <= 31 && mo > 0 && mo <= 12) {
                    year = (y < 80) ? (2000 + y) : (1900 + y);
                    month = mo;
                    day = d;
                    date_valid = 1;
                }
            }
            
            field++;
            field_start = i + 1;
            
            if (rmc_data[i] == '*') break;
        }
    }
    
    if (time_valid && date_valid) {
        g_disc.rmc_utc_ms = time_ms;
        g_disc.rmc_local_ms = nl_get_sys_ms();
        g_disc.year = year;
        g_disc.month = month;
        g_disc.day = day;
        g_disc.date_valid = 1;
        
        NL_LOG_D("RMC parsed: %04d-%02d-%02d %02d:%02d:%02d.%03d UTC", 
                 year, month, day, 
                 (int)(time_ms/3600000), (int)((time_ms%3600000)/60000), 
                 (int)((time_ms%60000)/1000), (int)(time_ms%1000));
        return 1;
    }
    
    return 0;
}

/**
 * @brief Process RMC data for coarse time synchronization
 */
int nl_gps_time_input_rmc(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        uint8_t c = buf[i];
        
        // Start of new NMEA sentence
        if (c == '$') {
            g_nmea_len = 0;
        }
        
        // Buffer the character
        if (g_nmea_len < sizeof(g_nmea_buf) - 1) {
            g_nmea_buf[g_nmea_len++] = c;
        }
        
        // End of NMEA sentence
        if (c == '\n' && g_nmea_len > 10) {
            g_nmea_buf[g_nmea_len] = '\0';  // Null terminate
            
            // Check for RMC message
            if (g_nmea_len >= 15 && 
                g_nmea_buf[0] == '$' && 
                g_nmea_buf[3] == 'R' && g_nmea_buf[4] == 'M' && g_nmea_buf[5] == 'C') {
                
                // Validate checksum
                if (validate_nmea_checksum(g_nmea_buf, g_nmea_len)) {
                    uint32_t now = nl_get_sys_ms();
                    
                    // Use RMC for coarse sync only in specific conditions
                    if (g_disc.state == DISC_INIT) {
                        if (parse_rmc_time_date(g_nmea_buf, g_nmea_len)) {
                            g_disc.last_rmc_update = now;
                            update_discipline_state();
                            return 1;
                        }
                    }
                } else {
                    NL_LOG_W("RMC checksum failed");
                }
            }
            
            g_nmea_len = 0;
        }
    }
    
    return 0;
}

/**
 * @brief Input UTC time directly (alternative to RMC)
 */
int nl_gps_time_input_utc(uint16_t year, uint8_t month, uint8_t day, 
                          uint8_t hour, uint8_t minute, uint8_t second, uint16_t ms)
{
    // Validate input parameters
    if (year < 1900 || year > 2100 || 
        month < 1 || month > 12 ||
        day < 1 || day > 31 ||
        hour >= 24 || minute >= 60 || second >= 60 || ms >= 1000) {
        return -1;
    }
    
    uint32_t now = nl_get_sys_ms();
    
    // Use for coarse sync only in specific conditions
    if (g_disc.state == DISC_INIT) {
        
        g_disc.rmc_utc_ms = hour * 3600000 + minute * 60000 + second * 1000 + ms;
        g_disc.rmc_local_ms = now;
        g_disc.last_rmc_update = now;
        
        g_disc.year = year;
        g_disc.month = month;
        g_disc.day = day;
        g_disc.date_valid = 1;
        
        NL_LOG_D("UTC input: %04d-%02d-%02d %02d:%02d:%02d.%03d", 
                 year, month, day, hour, minute, second, ms);
        
        update_discipline_state();
        return 0;
    }
    
    return 0;  // Ignored due to timing constraints
}

/**
 * @brief PPS callback - align to precise UTC second boundary
 */
void nl_gps_time_pps_callback(void)
{
    static uint32_t s_last_proc_ms = 0;
    
    uint32_t pps_time = nl_get_sys_ms();

    // Throttle: ignore too-frequent calls (< PPS_INTERVAL_MIN)
    if (s_last_proc_ms && (pps_time - s_last_proc_ms) < PPS_INTERVAL_MIN) return;
    s_last_proc_ms = pps_time;

    // Log after throttling to avoid spamming when line is noisy/high-rate
    NL_LOG_D("PPS edge at %u ms, state:%d", pps_time, g_disc.state);

    // Discard PPS in INIT state - no valid time reference yet
    if (g_disc.state == DISC_INIT) {
        NL_LOG_D("PPS ignored in INIT state");
        return;
    }

    // First valid PPS after entering COARSE state - establish baseline
    if (g_disc.state >= DISC_COARSE && g_disc.pps_count == 0) {
        g_disc.pps_count = 1;
        g_disc.valid_pps_count = 0;
        g_disc.last_pps_time = pps_time;
        g_disc.pps_unstable_count = PPS_STABILIZE_COUNT;
        NL_LOG_I("First valid PPS at %u ms after RMC", pps_time);
        
        // Attempt UTC second alignment using RMC timestamp
        if (g_disc.date_valid && g_disc.rmc_utc_ms > 0) {
            uint32_t rmc_age_ms = pps_time - g_disc.rmc_local_ms;
            if (rmc_age_ms < 2000) {
                // RMC reports the previous second; PPS marks the start of the current second
                uint32_t current_rmc_estimate = g_disc.rmc_utc_ms + rmc_age_ms;
                uint32_t rmc_second = (current_rmc_estimate / 1000) % 86400;
                g_disc.pps_utc_second = (rmc_second + 1) % 86400;
                g_disc.pps_local_ms = pps_time;
                NL_LOG_I("PPS initial alignment: RMC=%u, PPS=%u", rmc_second, g_disc.pps_utc_second);
            }
        }
        
        update_discipline_state();
        return;
    }

    // Subsequent PPS - validate interval against last valid PPS
    uint32_t interval = pps_time - g_disc.last_pps_time;

    // Reject PPS with invalid interval
    if (interval > PPS_INTERVAL_MAX || interval < PPS_INTERVAL_MIN) {
        NL_LOG_W("Invalid PPS interval: %u ms", interval);
        
        // Large gap detected - force re-stabilization
        if (interval > 1500) {
            g_disc.pps_unstable_count = PPS_STABILIZE_COUNT;
            NL_LOG_W("PPS gap detected: %u ms - entering stabilization", interval);
        }
        
        // Reset valid count but update baseline to current PPS
        g_disc.valid_pps_count = 0;
        g_disc.last_pps_time = pps_time;
        update_discipline_state();
        return;
    }

    // During stabilization, reject pulses with large deviation from 1000 ms
    if (g_disc.pps_unstable_count > 0) {
        if (abs((int32_t)interval - 1000) > 50) {
            NL_LOG_W("PPS glitch during stabilization: %u ms - restarting", interval);
            // Restart stabilization on glitch
            g_disc.pps_unstable_count = PPS_STABILIZE_COUNT;
            g_disc.valid_pps_count = 0;
            g_disc.last_pps_time = pps_time;
            update_discipline_state();
            return;
        }
        g_disc.pps_unstable_count--;
        NL_LOG_D("PPS stabilizing: %d remaining", g_disc.pps_unstable_count);
    }

    // Valid PPS - update counters and baseline
    g_disc.valid_pps_count++;
    g_disc.pps_count++;
    g_disc.last_pps_time = pps_time;

    // UTC second alignment - only when stable and coarse time is available
    if (g_disc.date_valid && g_disc.rmc_utc_ms > 0 && g_disc.pps_unstable_count == 0) {
        if (g_disc.pps_utc_second == 0) {
            // Should have been initialized on first valid PPS, but handle edge case
            uint32_t current_rmc_estimate = g_disc.rmc_utc_ms + (pps_time - g_disc.rmc_local_ms);
            uint32_t rmc_second = (current_rmc_estimate / 1000) % 86400;
            g_disc.pps_utc_second = (rmc_second + 1) % 86400;
            g_disc.pps_local_ms = pps_time;
            NL_LOG_I("PPS late alignment: RMC=%u, PPS=%u", rmc_second, g_disc.pps_utc_second);
        } else {
            // Increment UTC second based on time elapsed since last PPS
            uint32_t time_gap_ms = pps_time - g_disc.pps_local_ms;
            uint32_t expected_seconds = (time_gap_ms + 500) / 1000;

            if (expected_seconds >= 1 && expected_seconds <= 10) {
                g_disc.pps_utc_second = (g_disc.pps_utc_second + expected_seconds) % 86400;
                g_disc.pps_local_ms = pps_time;
            } else if (expected_seconds > 10) {
                // Large gap realignment with +1 second correction
                uint32_t current_rmc_estimate = g_disc.rmc_utc_ms + (pps_time - g_disc.rmc_local_ms);
                uint32_t rmc_second = (current_rmc_estimate / 1000) % 86400;
                g_disc.pps_utc_second = (rmc_second + 1) % 86400;
                g_disc.pps_local_ms = pps_time;
                NL_LOG_W("PPS large gap: RMC=%u, realigned PPS=%u", rmc_second, g_disc.pps_utc_second);
            }
        }
    }

    update_discipline_state();
}


/**
 * @brief Update disciplining state machine
 */
static void update_discipline_state(void)
{
    uint32_t now = nl_get_sys_ms();
    discipline_state_t old_state = g_disc.state;
    
    switch (g_disc.state) {
        case DISC_INIT:
            if (g_disc.date_valid && g_disc.rmc_utc_ms > 0) {
                g_disc.state = DISC_COARSE;
                NL_LOG_I("Coarse time acquired from RMC");
            }
            break;
            
        case DISC_COARSE:
            if (g_disc.valid_pps_count >= PPS_LOCK_COUNT && g_disc.pps_unstable_count == 0) {
                g_disc.state = DISC_LOCKED;
                NL_LOG_I("PPS locked - precise timing available");
            }
            break;
            
        case DISC_LOCKED:

            if (now - g_disc.last_pps_time > PPS_TIMEOUT_MS) {
                g_disc.state = DISC_INIT;
                g_disc.valid_pps_count = 0;

                NL_LOG_W("PPS timeout - back to init (time preserved)");
            }
            break;
    }
    
    if (old_state != g_disc.state) {
        g_disc.state_enter_time = now;
    }
}

/**
 * @brief Get current time as daily milliseconds
 * @return Daily milliseconds (0-86399999) or 0 if not synchronized
 */
uint32_t nl_gps_time_get_daily_ms(void)
{
    update_discipline_state();
    
    if (g_disc.state < DISC_COARSE && g_disc.rmc_utc_ms == 0) {
        return 0;
    }
    
    uint32_t now = nl_get_sys_ms();
    uint32_t daily_ms = 0;
    
    if (g_disc.state == DISC_LOCKED) {

        uint32_t elapsed_since_pps = now - g_disc.pps_local_ms;
        daily_ms = (g_disc.pps_utc_second * 1000) + elapsed_since_pps;
    } 
    else if (g_disc.pps_utc_second > 0 && g_disc.pps_local_ms > 0) {
        uint32_t elapsed_since_pps = now - g_disc.pps_local_ms;
        daily_ms = (g_disc.pps_utc_second * 1000) + elapsed_since_pps;
    }
    
    return daily_ms % 86400000;
}

/**
 * @brief Get current UTC time
 */
int nl_gps_time_get_utc(gps_utc_time_t *utc)
{
    if (!utc || !g_disc.date_valid) {
        return -1;
    }
    
    uint32_t daily_ms = nl_gps_time_get_daily_ms();
    if (daily_ms == 0) {
        return -1;
    }
    
    // Fill date components
    utc->year = g_disc.year;
    utc->month = g_disc.month;
    utc->day = g_disc.day;
    
    // Convert daily milliseconds to time components
    utc->ms = daily_ms % 1000;
    uint32_t total_seconds = daily_ms / 1000;
    
    utc->hour = total_seconds / 3600;
    uint32_t remainder = total_seconds % 3600;
    utc->minute = remainder / 60;
    utc->second = remainder % 60;
    
    return 0;
}


/**
 * @brief Check if GPS time is locked (PPS synchronized)
 */
int nl_gps_time_is_locked(void)
{
    update_discipline_state();
    return (g_disc.state >= DISC_LOCKED) ? 1 : 0;
}
