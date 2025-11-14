

#include "../include/wdt.h"
#include <msp432p401r.h>


/// @brief Initializes the Watchdog Timer
void WDT_init(struct wdt* wdt_instance, unsigned long base, const struct wdt_config_t* config) { 

    wdt_instance->base = base;

    WDT_A_Type* regs = (WDT_A_Type*)base;


    regs->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop WDT during configuration


    uint16_t ctl =
        WDT_A_CTL_PW       |   // password ALWAYS required
        config->mode_select |
        config->clock_source |
        config->counter_clear |
        config->interval_select;

    wdt_instance->ctl = ctl;   // save desired configuration

    regs->CTL = ctl;           // atomic write    
}


/// @brief Holds (stops) the Watchdog Timer
void WDT_hold(struct wdt* wdt_instance) { 

    WDT_A_Type* regs = (WDT_A_Type*)wdt_instance->base;

    regs->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop WDT
}