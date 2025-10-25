

#include <stdint.h>
#include <msp432p401r.h>
#include "../include/uart.h"

// Clock Frequency Defines
#define ACLK_FREQ_HZ (32768U)
#define SMCLK_FREQ_HZ (3000000U)


// UART transmit functions
void sendString(char *str);
void sendChar(char s);


// UART configurations 
// Clock frequency:  3MHz (SMCLK)
// Baud rate: 9600 
static const UART_config_t UART_A0_config = {
    .parity = UART_PARITY_NONE, 
    .order  = UART_LSB_FIRST, 
    .data_length = UART_DATA_8BIT, 
    .mode = UART_MODE, 
    .clock_sel = UART_SMCLK, 
    .baud_rate = 9600, 
    .oversampling = UART_OVERSAMPLING_ON,
    .baud_prescaler = 19,
    .firstMod  = 9,
    .secondMod = 0xAA
};


static const UART_config_t UART_A2_config = {
    .parity = UART_PARITY_NONE, 
    .order  = UART_LSB_FIRST, 
    .data_length = UART_DATA_8BIT, 
    .mode = UART_MODE, 
    .clock_sel = UART_SMCLK, 
    .baud_rate = 9600, 
    .oversampling = UART_OVERSAMPLING_ON,
    .baud_prescaler = 19,
    .firstMod  = 9,
    .secondMod = 0xAA
};


int main(void)
{

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    // Enable UART0 Pins
    // P1.2->RX
    // P1.3->TX
    P1->SEL0 |= BIT2 | BIT3;
    P1->SEL1 &= ~(BIT2 | BIT3);

    // UART0 Configuration
    // Enhanced Universal Serial Control Interface = EUSCI

    // Add the configuration setup code here
    UART_initModule(EUSCI_A0, &UART_A0_config); 
    UART_enableModule(EUSCI_A0); 

    // Enable UART2 Pins
    // P3.2->RX
    // P3.3->TX
    P3->SEL0 |= BIT2 | BIT3;
    P3->SEL1 &= ~(BIT2 | BIT3);

    // UART2 Configuration
    // Add the configuration setup code here and enable interrupt code
    UART_initModule(EUSCI_A2, &UART_A2_config); 
    UART_enableModule(EUSCI_A2); 


    // Enable Interrupts 
    UART_enableInterrupts(EUSCI_A0, EUSCI_A_IE_RXIE);
    

    // enable NVIC for UART0
    NVIC->ISER[0] = 1 << (EUSCIA0_IRQn & 31);
    // enable global interrupts
    __enable_irq();


    sendString("Enter r for red, g for green, b for blue!\r\n"); // send message

    while (1)
    {
        // do nothing
    }
}

void EUSCIA0_IRQHandler(void)
{
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG) // receive interrupt
    {
        char c = EUSCI_A0->RXBUF; // store data into character buffer, and clear flag
        sendChar(c);              // display character through the SERIAL port
    }
}

void sendString(char *str)
{
    int i = 0;
    while (str[i] != '\0')
    {

        // Add the condition inside while loop
        while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG)); // wait till TXBUF is empty (Check TXIFG flag)
        EUSCI_A0->TXBUF = str[i]; // send character through buffer
        i++;
    }
}

void sendChar(char s)
{
    // Echo character sent back to host PC serial monitor
    // Add the condition inside while loop
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG)); // wait till TXBUF is empty (Check TXIFG flag)
        EUSCI_A0->TXBUF = s; // send character through buffer

    while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG)); // wait till TXBUF is empty (Check TXIFG flag)
    EUSCI_A2->TXBUF = s; // send character through buffer
}


