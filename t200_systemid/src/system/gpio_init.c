

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "../../include/system/gpio_init.h"


// Hall Effect Sensor Sampling pin 
static const uint8_t HALL_PIN = GPIO_PIN1; // P6.1 (ADC)


// UART pin definition 
static const uint8_t RX_PIN = GPIO_PIN2; //  P1.2 (RX)
static const uint8_t TX_PIN = GPIO_PIN3; //  P1.3 (TX)


// T200 ESC PWM Pin definition
static const uint8_t PWM_PIN = GPIO_PIN5; // P2.5

/// @brief Initializes all GPIO Pins used in application
void gpio_init()
{
    // Set GPIO pins as ADC input
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, HALL_PIN, GPIO_TERTIARY_MODULE_FUNCTION);
    
    // Configure P3.2 (TX) and P3.3 (RX) as UART pins
    //GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, RX_PIN | TX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    // Configure P1.2 (RX) and P1.3 (TX)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            RX_PIN | TX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    
    // Configure P2.5 to generate PWM to T200 thruster
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, PWM_PIN,
                 GPIO_PRIMARY_MODULE_FUNCTION);
}

