#include "app_pmux.h"
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG    "io.pmux"
#define DBG_LVL    DBG_ERROR
#include <rtdbg.h>

// MUX configuration storage
static app_pmux_config_t mux_configs[IO_MUX_MAX];
static const uint8_t *current_gpio_map = RT_NULL;

// Convert IO pin number to GPIO pin number
static uint8_t io_to_gpio_pin(uint8_t io_pin)
{
    if (!current_gpio_map || io_pin >= IO_MUX_MAX) {
        return IO_MUX_INVALID;
    }
    return current_gpio_map[io_pin];
}

// Find MUX configuration by ID
static app_pmux_config_t* find_mux_config(uint8_t mux_id)
{
    if (mux_id >= IO_MUX_MAX) {
        return RT_NULL;
    }
    return &mux_configs[mux_id];
}

// Disable GPIO pin for specific MUX (reset to safe state)
static void disable_mux_pin(app_pmux_config_t *config)
{
    if (config->gpio_pin != IO_MUX_INVALID) {
        // Reset pin to input mode as safe state
        rt_pin_mode(config->gpio_pin, PIN_MODE_INPUT);
        LOG_I("MUX%d GPIO%d disabled", config->mux_id, config->gpio_pin);
        config->gpio_pin = IO_MUX_INVALID;
    }
}

void app_pmux_init(const uint8_t *gpio_map)
{
    if (!gpio_map) {
        LOG_E("Invalid GPIO map");
        return;
    }
    
    current_gpio_map = gpio_map;
    
    // Initialize all MUX configurations to invalid state
    for (int i = 0; i < IO_MUX_MAX; i++) {
        mux_configs[i].mux_id = i;
        mux_configs[i].gpio_pin = IO_MUX_INVALID;
        mux_configs[i].mode = PIN_MODE_INPUT;
    }
    
    LOG_I("IO PMUX initialized");
}

int app_pmux_config(uint8_t mux_id, uint8_t io_pin)
{
    if (mux_id >= IO_MUX_MAX) {
        LOG_E("Invalid MUX ID: %d", mux_id);
        return -RT_EINVAL;
    }
    
    uint8_t gpio_pin = io_to_gpio_pin(io_pin);
    
    if (gpio_pin == IO_MUX_INVALID) {
        LOG_W("Invalid IO pin: %d", io_pin);
        return -RT_EINVAL;
    }
    
    app_pmux_config_t* config = find_mux_config(mux_id);
    
    // PMUX override mechanism: disable previous GPIO if switching to different pin
    if (config->gpio_pin != IO_MUX_INVALID && config->gpio_pin != gpio_pin) {
        LOG_I("MUX%d switching from GPIO%d to GPIO%d", mux_id, config->gpio_pin, gpio_pin);
        disable_mux_pin(config);
    }
    
    // Check if other MUX is already using this GPIO and disable it
    for (int i = 0; i < IO_MUX_MAX; i++) {
        if (i != mux_id && mux_configs[i].gpio_pin == gpio_pin) {
            LOG_W("GPIO%d was used by MUX%d, now switching to MUX%d",  gpio_pin, i, mux_id);
            disable_mux_pin(&mux_configs[i]);
        }
    }
    
    // Configure new MUX mapping
    config->gpio_pin = gpio_pin;
    
    // Set default pin mode based on MUX type
    switch(mux_id) {
        case IO_MUX1_SIN:
            config->mode = PIN_MODE_INPUT_PULLDOWN;
            break;
        case IO_MUX2_SOUT:
        case IO_MUX3_LED_RUN:
        case IO_MUX4_LED_NETWORK:
        case IO_MUX9_TEST:
            config->mode = PIN_MODE_OUTPUT;
            break;
        default:
            config->mode = PIN_MODE_INPUT;
            break;
    }
    
    // Apply pin mode configuration
    rt_pin_mode(gpio_pin, config->mode);
    
    LOG_I("MUX%d configured to IO%d (GPIO%d), mode=%d", mux_id, io_pin, gpio_pin, config->mode);
    return RT_EOK;
}

uint8_t app_pmux_get_pin(uint8_t mux_id)
{
    app_pmux_config_t* config = find_mux_config(mux_id);
    return config ? config->gpio_pin : IO_MUX_INVALID;
}
