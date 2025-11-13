
    /*
     * MSP432P401R - Low Power Mode 3 (LPM3) Lab Experiment
     *
     * The code has functions for entering low power mode and exiting into active mode.
     *
     * In both the modes, (P1.4) is set as an interrupt source triggered on falling edge.
     * The interrupt handler PORT1_IRQHandler() toggles a flag that commands the
     * system power state.
     *
     * GPIO Configuration:
     *  -All ports are set as GPIOs with output mode and pull down enabled except:
     *  -PJSEL0 and PJSEL1 are LFXTCLK pins and must be configured as clock pins
     *  -(P1.4) set as an interrupt to change modes
     *  -(P1.0) set as an output
     *
     *              +-------------+
     *              | Active Mode |
     *              +-------------+
     * (WDT_A) the Watchdog Timer-A should be configured:
     *  -as an interval timer with interval of 1s
     *  -clock source should be X_CLK
     *  -interrupt handler (WDT_A_IRQHandler) should toggle LED-(P1.0)
     *
     * (CS) Clock System Clock configuration:
     *  -all clocks enabled (ACLK, MCLK, HSMCLK, SMCLK)
     *  -DCOCLK enabled and selected as clock source
     *
     * (PSS) Power Supply System configuration:
     *  -turn on supply voltage supervisor and monitor (SVSMH)
     *
     *              +-------------+
     *              |  LPM3 Mode  |
     *              +-------------+
     * (WDT_A) Watchdog timer is halted.
     *
     * (CS) Clock System Clock configuration:
     *  -all clocks disabled (ACLK, MCLK, HSMCLK, SMCLK)
     *  -LFXTCLK enabled and selected as clock source
     *
     * (PSS) Power Supply System configuration:
     *  -turn off supply voltage supervisor and monitor (SVSMH)
     *
     */



#include <stdint.h>
#include <stdbool.h>
#include <msp432p401r.h>
#include "../include/wdt.h"

#define false 0
#define true 1
#define MAX_ERROR_COUNT 50

void ConfigureGPIO(void);
void ConfigureWDT(void);
_Bool EnterActiveMode(void);
_Bool EnterLowPowerMode(void);


volatile _Bool low_pwr_state = false;
volatile _Bool state_change = false;
_Bool valid_state_change_to_lpm = true;
_Bool valid_state_change_to_am = true;
uint32_t counter = 0;



// Watchdog Timer A instance and configuration
struct wdt wdt_a;

static const struct wdt_config_t wdt_config_interval_timer_1s = {
    .mode_select = WDT_A_CTL_TMSEL, // Timer Interval Mode
    .interval_select = WDT_A_CTL_IS_4,
    .clock_source = WDT_A_CTL_SSEL_3,
    .counter_clear = WDT_A_CTL_CNTCL
};



void main(void)
{
    WDT_hold(&wdt_a);
    //WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog

    ConfigureGPIO();
    EnterActiveMode();

    while(1){
        if(state_change){
            if(low_pwr_state){
                valid_state_change_to_lpm = EnterLowPowerMode();
                low_pwr_state = valid_state_change_to_lpm;
            }
            else{
                valid_state_change_to_am = EnterActiveMode();
               low_pwr_state = !valid_state_change_to_am;
            }
            state_change = false;

            if(!(valid_state_change_to_lpm & valid_state_change_to_am)){
                WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_SSEL_3 | WDT_A_CTL_TMSEL | WDT_A_CTL_CNTCL | WDT_A_CTL_IS_5;
            }
        }
        if(low_pwr_state)
            __WFI(); // enter sleep and wait for interrupt
    }
}

void ConfigureWDT(void)
{
    /********************* ADD CODE HERE *****************************
     * Configure the WDT as an Interval Timer
     * Set the interval to 1s
     * Select X_CLOCK as the source
     * Enable WDT_A_IRQn interrupt in NVIC
     *
     * Example WDT configuration ( 250ms interval timer )
     *   WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_SSEL_3 | WDT_A_CTL_TMSEL | WDT_A_CTL_CNTCL | WDT_A_CTL_IS_5;
     */

    // Add your lines of code here >>
    WDT_init(&wdt_a, WDT_A_BASE, &wdt_config_interval_timer_1s);
    NVIC_EnableIRQ(WDT_A_IRQn);

    // end here
}

void WDT_A_IRQHandler(void) {
    /********************* ADD CODE HERE *****************************
     * Toggle state of LED P1.0
     */

    if(!low_pwr_state){     // Blink LED in Active Mode

        // Add your lines of code here >>
       
        // end here

    }
}

void PORT1_IRQHandler(void)
{
  if(P1->IFG & BIT4){
    state_change = true; // Move function calls out of ISR

    if(low_pwr_state)
        low_pwr_state = false;
    else
        low_pwr_state = true;

    P1->IFG &= ~BIT4;  // Clear the P1.4 interrupt flag
  }
}

// Configure all the ports as GPIOs set in output mode
// with pull-down resistors enabled.
void ConfigureGPIO(void)
{
    PA -> SEL0 = 0x00; PA -> SEL1 = 0x00;
    PB -> SEL0 = 0x00; PB -> SEL1 = 0x00;
    PC -> SEL0 = 0x00; PC -> SEL1 = 0x00;
    PD -> SEL0 = 0x00; PD -> SEL1 = 0x00;
    PE -> SEL0 = 0x00; PE -> SEL1 = 0x00;

    PA -> REN |= 0xFFFF; PA -> DIR |= 0xFFFF; PA -> OUT &= 0x0000;
    PB -> REN |= 0xFFFF; PB -> DIR |= 0xFFFF; PB -> OUT &= 0x0000;
    PC -> REN |= 0xFFFF; PC -> DIR |= 0xFFFF; PC -> OUT &= 0x0000;
    PD -> REN |= 0xFFFF; PD -> DIR |= 0xFFFF; PD -> OUT &= 0x0000;
    PE -> REN |= 0xFFFF; PE -> REN |= 0xFFFF;  PE -> OUT &= 0x0000;

    P1->DIR |= BIT0; // Setup LED 1.0
    P1->OUT = 0;
    P1->REN &= ~BIT0;

    P1->DIR &= ~BIT4; // Input on P1.4
    P1->REN |= BIT4; // Enable pull up
    P1->OUT |= BIT4;
    P1->IES |= BIT4;   // Set interrupt edge select (falling edge)
    P1->IFG &= ~BIT4;  // Clear interrupt flag
    P1->IE |= BIT4;    // Enable interrupt

    NVIC_EnableIRQ(PORT1_IRQn);  // Enable Port1 interrupt in the NVIC
}

_Bool EnterActiveMode(void)
{
    ConfigureWDT(); // Configure WDT to toggle P1.0

    SCB->SCR &= ~(1 << 2); // SLEEPDEEP bit cleared

    // clear force LPM entry
    PCM->CTL1 = PCM_CTL1_KEY_VAL & ~PCM_CTL1_FORCE_LPM_ENTRY;
    PCM->CLRIFG |= PCM_CLRIFG_CLR_AM_INVALID_TR_IFG;

    while(PCM->CTL1 & PCM_CTL1_PMR_BUSY){;} // wait until PCM is free

    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR__AM_LDO_VCORE0; // set AM LDO VC0

    counter = 0;
    while(PCM->IFG & PCM_IFG_AM_INVALID_TR_IFG){
        if(counter < MAX_ERROR_COUNT){
            PCM->CLRIFG |= PCM_CLRIFG_CLR_AM_INVALID_TR_IFG;
            counter++;
        }
        else{
            return false;
        }
    }

    while(PCM->CTL1 & PCM_CTL1_PMR_BUSY){;}

    CS->KEY = CS_KEY_VAL; // unlock CS access
    // enable all clocks and select DCOCLK as source
    CS->CLKEN |= CS_CLKEN_ACLK_EN | CS_CLKEN_MCLK_EN | CS_CLKEN_HSMCLK_EN | CS_CLKEN_SMCLK_EN;
    CS->CTL1 |= CS_CTL1_SELM_3 | CS_CTL1_SELS_3;
    CS->KEY = 0; // lock access to CS registers

    PSS ->KEY = PSS_KEY_KEY_VAL; // access PSS
    PSS ->CTL0 &= ~PSS_CTL0_SVSMHOFF; // Turn ON SVSM
    PSS ->KEY = 1; // lock PSS access

    return true;
}

_Bool EnterLowPowerMode(void)
{
    // Disable WDT
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    PJ->SEL0 |= 0x03; // Set PJSEL for LFXT operation
    PJ->SEL1 &= ~0x03;

    P1->OUT &= ~BIT0;

    // Turn off Supply Voltage Supervisor (SVSMH)
    PSS ->KEY = PSS_KEY_KEY_VAL; // access PSS
    PSS ->CTL0 |= PSS_CTL0_SVSMHOFF; // Turn OFF SVSM
    PSS ->KEY = 1; // lock PSS access

    CS->CLRIFG |= CS_CLRIFG_CLR_LFXTIFG;
    // Turn off all clocks except LFXTCLK
    CS->KEY = CS_KEY_VAL;   // enable access to CS registers
    CS->CTL2 = CS_CTL2_LFXT_EN; // Enable low frequency clock
    CS->CTL1 = 0; // select LFXTCLK source

    // Turn off ACLK,MCLK,HSMCLK,SMCLK
    CS->CLKEN &= ~(CS_CLKEN_ACLK_EN | CS_CLKEN_MCLK_EN | CS_CLKEN_HSMCLK_EN | CS_CLKEN_SMCLK_EN);

    CS->KEY = 0; // lock access to CS registers

    while(PCM->CTL1 & PCM_CTL1_PMR_BUSY){;}

    PCM->CLRIFG |= PCM_CLRIFG_CLR_LPM_INVALID_TR_IFG;
    SCB -> SCR |= (1 << 2);  // allow deep sleep
    PCM->CTL1 = PCM_CTL1_KEY_VAL | PCM_CTL1_FORCE_LPM_ENTRY; // force LPM3 entry
    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_LPMR__LPM3; // Request LPM3

    counter = 0;
    while(PCM->IFG & PCM_IFG_LPM_INVALID_TR_IFG){
        if(counter < MAX_ERROR_COUNT){
            PCM->CLRIFG |= PCM_CLRIFG_CLR_LPM_INVALID_TR_IFG;
            counter++;
        }
        else{
            return false;
        }
    }

    while(PCM->CTL1 & PCM_CTL1_PMR_BUSY){;} // wait until PCM is free

    return true;
}
