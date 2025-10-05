

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "../../include/system/timer_init.h"
#include "../../include/app/tables.h"



// This is all current hardcoded to 400Hz. In future we want to be able to configure this externally
static const uint_fast16_t pwm_period_400Hz_us = 2500; 
static const uint_fast16_t pwm_period_400Hz_CCR = 29999;
static const uint_fast16_t stop_signal_us = 1500;


Timer_A_PWMConfig pwmConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        pwm_period_400Hz_CCR,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        (stop_signal_us/pwm_period_400Hz_us)*pwm_period_400Hz_CCR                           // By default T200 should be off
        
};


// The timer is configured to interrupt at 12.8kHz to update the duty cycle in
// sin_table at time steps of 78.125uS. 
// CCR0 = SMCLK frequency / target frequency -1
// CCR0 = 12Mhz / 12.8kHz - 1 = 937
static const uint16_t updatePeriod = 937;  

static const Timer_A_UpModeConfig updateTableConfig = 
{
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_1,
    updatePeriod, 
    TIMER_A_TAIE_INTERRUPT_DISABLE,     // Don't enable this one before it raises interrupt after counter resets to 0
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Raise interrupt to happen when counter == CCR0  
    TIMER_A_DO_CLEAR
};




/* Function Prototypes */

void timer_init() { 

    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig);
    // This starts the timer that will update the duty cycle 
    Timer_A_configureUpMode(TIMER_A0_BASE, &updateTableConfig);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

}

void TA0_0_IRQHandler(void)
{

    // Clear the interrupt flag and reset timer count
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    pwmConfig.dutyCycle = convert_to_duty(pwm_period_400Hz_us, pwm_period_400Hz_CCR, sin_table[tbl_index]);

    Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, pwmConfig.dutyCycle);
    
    tbl_index++;

    // Wrap around when end of table is reached.
    if (tbl_index >= TABLE_SIZE)
        tbl_index = 0;
}

uint_fast16_t convert_to_duty(const uint_fast16_t pwm_period_us, const uint_fast16_t ccr, const uint_fast16_t pulsewidth_us) {
    return (pulsewidth_us * ccr) / pwm_period_us;
}