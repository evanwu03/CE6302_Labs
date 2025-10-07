

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "../../include/system/gpio_init.h"



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
    
    // Configure P2.4 to generate PWM to T200 thruster
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, PWM_PIN,
                 GPIO_PRIMARY_MODULE_FUNCTION);
    
    // Debug  pin
    GPIO_setAsOutputPin(GPIO_PORT_P6, DEBUG_PIN);

    

}

