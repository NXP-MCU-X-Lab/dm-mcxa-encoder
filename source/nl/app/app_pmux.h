#ifndef __IO_PMUX_H__
#define __IO_PMUX_H__

#include <stdint.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    IO_MUX1_SIN = 1,
    IO_MUX2_SOUT = 2,
    IO_MUX3_LED_RUN = 3,
    IO_MUX4_LED_NETWORK = 4,
    IO_MUX7_SENSOR_RST = 7,
    IO_MUX9_TEST = 9,
    IO_MUX_MAX = 10,
    IO_MUX_INVALID = 255,
};


// MUX configuration structure
typedef struct {
    uint8_t mux_id;
    uint8_t gpio_pin;
    uint8_t mode;
} app_pmux_config_t;

// Function declarations
void app_pmux_init(const uint8_t *gpio_map);
int app_pmux_config(uint8_t mux_id, uint8_t io_pin);
uint8_t app_pmux_get_pin(uint8_t mux_id);

#ifdef __cplusplus
}
#endif

#endif /* __IO_PMUX_H__ */
