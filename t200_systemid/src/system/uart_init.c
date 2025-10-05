
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "../../include/system/uart_init.h"


/// @brief Initializes UART peripheral
void uart_init()
{


    // Initialize UART module A0 with local config
    UART_initModule(EUSCI_A0_BASE, &uartConfig);

    // Enable UART module
    UART_enableModule(EUSCI_A0_BASE);

    //UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);

}