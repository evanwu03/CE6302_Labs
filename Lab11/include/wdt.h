
#ifndef WDT_H
#define WDT_H

#include <stdint.h>


/// @brief Watchdog Timer instance structure
struct wdt{
    uintptr_t base;
    uint16_t ctl
;};


/// @brief Configuration structure for the Watchdog Timer
/// Contains settings for mode, interval, clock source, and counter clear
struct wdt_config_t {
    uint8_t mode_select;
    uint8_t interval_select;
    uint8_t clock_source;
    uint8_t counter_clear;
};

/// @brief Initializes the Watchdog Timer
void WDT_init(struct wdt* wdt_instance, unsigned long base, const struct wdt_config_t* config);

/// @brief Holds (stops) the Watchdog Timer
void WDT_hold(struct wdt* wdt_instance);


#endif 
