
#include "msp.h"
//#include <msp432.h>"

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    // Error LEDs
    P1->DIR |= BIT0;    // set BIT0 as OUTPUT
    P1->OUT &= ~(BIT0); // set BIT0 as LOW
    //clear secondary functions
    P1->SEL0 &= ~(BIT0);
    P1->SEL1 &= ~(BIT0);

    // RGB LEDs (LED2)
    P2->DIR |= BIT0 | BIT1 | BIT2;    // set BIT0 (Red), BIT1 (Green), BIT2 (Blue) as OUTPUTs
    P2->OUT &= ~(BIT0 | BIT1 | BIT2); // set BIT0,1,2 as LOW
    // clear secondary functions
    P2->SEL0 &= ~(BIT0 | BIT1 | BIT2);
    P2->SEL1 &= ~(BIT0 | BIT1 | BIT2);

    // Enable UART2 Pins
    // P3.2->RX
    // P3.3->TX
    P3->SEL0 |= BIT2 | BIT3; 
    P3->SEL1 &= ~(BIT2 | BIT3);

    // UART2 Configuration
    // Enhanced Universal Serial Control Interface = EUSCI
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST; // Clear previous configuration of UART


    
    // Add the configuration code here and enable UART interrupt
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK; // Use SMCLK as clock source
    
    // Baud Rate 9600, BRCLK = 1MHz
    EUSCI_A2->BRW = 19; // UCBRX = 6
    EUSCI_A2->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS) |  // UCBRFX = 8
                      (0xAA << EUSCI_A_MCTLW_BRS_OFS) | // UCBRSX = 0x20
                      EUSCI_A_MCTLW_OS16;               // UCOS16 = 1 (Oversampling)




    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Release from reset
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;         // Enable RX interrupt

    // enable NVIC for UART2
    NVIC->ISER[0] = 1 << (EUSCIA2_IRQn & 31);

    // enable global interrupts
    __enable_irq();

    while (1); // Wait for interrupts
}

void EUSCIA2_IRQHandler(void)
{
    if (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG) // receive interrupt
    {
        // Add the logic here to receive string from UART transmitter and Toggle LED2 
        // according to the received character
        // if any other character received toggle error LED1

        char c = EUSCI_A2->RXBUF; // Read buffer to clear flag

        if (c == 'r')
        {
            P1->OUT &= ~(BIT0);
            P2->OUT &= ~(BIT1 | BIT2); // Turn off Green and Blue
            P2->OUT ^= BIT0;          // Toggle Red
        }
        else if (c == 'g') // Assumed from 'r' and 'b' in sender's message
        {
            P1->OUT &= ~(BIT0);
            P2->OUT &= ~(BIT0 | BIT2); // Turn off Red and Blue
            P2->OUT ^= BIT1;          // Toggle Green
        }
        else if (c == 'b') // Assumed from 'r' and 'g' in sender's message
        {
            P1->OUT &= ~(BIT0);
            P2->OUT &= ~(BIT0 | BIT1); // Turn off Red and Green
            P2->OUT ^= BIT2;          // Toggle Blue
        }
        else
        {
            // Blink error LED1 (P1.0) two times
            
            //P2->OUT &= ~(BIT0 | BIT 1 | BIT2);         // Ensure it's off
            //P1->OUT ^= BIT0;          // Toggle Blue

            P2->OUT &= ~(BIT0 | BIT1 | BIT2);  
            P1->OUT |= BIT0;

            //__delay_cycles(200000); // Simple delay
            //P1->OUT &= ~BIT0;
            //__delay_cycles(200000);
            
        }
    }
}
