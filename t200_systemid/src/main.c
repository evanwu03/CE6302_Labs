/* 
Purpose: Control a T200 thruster using a predefined square and sine wave of PWM values 
*/


// TI DriverLib Library
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "msp432.h"

// User-defined libraries
#include "../include/app/printf.h"
#include "../include/app/tables.h"
#include "../include/system/adc_init.h"
#include "../include/system/clock_init.h"
#include "../include/system/gpio_init.h"
#include "../include/system/timer_init.h"
#include "../include/system/uart_init.h"


// Raw current value from ADC (14bits: 0-16383)
volatile uint16_t rawCurrent;

// Global status flag
volatile uint8_t data_is_ready; // Check if data from ADC has been received and is ready to send

// Index to table, Note: can we make this not a global variable?
volatile uint8_t tbl_index = 0;

#define VCC (5U) // Voltage supply to ACS709 Hall effect sensor
static const float ADC_REF= 2.5; 


#define RINGBUF_SIZE (2048U)
volatile uint16_t ringbuf[RINGBUF_SIZE];
volatile uint16_t head = 0, tail = 0;


// Configuration mode 
#define DEBUG 
//#define RELEASE


int main(void)
{
    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);
    // Disable all interrupts
    Interrupt_disableMaster();

    // Configure System clock
    system_clock_init();

    // Configure pins for ADC input
    gpio_init();


    // Enable ADC module
    adc_init();

    // Enable Timer A 
    timer_init();

    // Enable UART module
    uart_init();

    // Enable interrupts after initializing
    Interrupt_enableInterrupt(INT_ADC14);
    Interrupt_enableInterrupt(INT_TA0_0);

    Interrupt_enableMaster();

    while (1)
    {

        

        if (data_is_ready)
        {
            printf(EUSCI_A0, "ADC: %u\n", rawCurrent);

            // Convert raw current to actual current in amps 
            // VIOUT = (0.028 V/A * i + 2.5 V) * VCC / 5 V
            float VIout = (rawCurrent * ADC_REF) / 16384;
            float Iout =  ((5*VIout/ VCC) - 2.5) / 0.028;   
            //printf(EUSCI_A0, "Iout: %i\n", Iout);

            data_is_ready = false; // reset flag
        }


        #ifdef DEBUG 
            
            if (head != tail) {
                uint16_t dutycycle = ringbuf[tail];
                tail = (tail + 1) % RINGBUF_SIZE;
                printf(EUSCI_A0, "Current duty cycle: %u\n", dutycycle); 
                //printf(EUSCI_A0, "UUUUU");
            }
           
        #endif 

        
        //PCM_gotoLPM0InterruptSafe(); // Go back to sleep
    }
}




