// MASTER TX ADDED LED TO SEE TRANSFER

#include <stdint.h>
#include <stdbool.h>
#include "../include/uart.h"
#include "../include/spi.h"
#include <msp432p401r.h>


#define PERIOD_1000_HZ 33 
#define DELAY_1000_MS  1000

#define PACKET_SIZE 4
uint8_t packet[PACKET_SIZE] = {0x7D, 0xAA, 0x33, 0x77};


volatile int packetIndex = 0;
volatile bool transmit_is_ready = false; 
bool led_is_on = false; 


// Global counter to keep track of time
volatile uint32_t millis = 0;
uint32_t prevTransmitTime = 0;

/* Parameters to Configure eUSCI B0 Module for SPI mode */
static const SPI_Config_t B0_SPI_config = { 
    .clock_sel = SPI_SMCLK,
    .mode = SPI_MASTER_MODE, 
    .data_order = SPI_LSB_FIRST, 
    .data_length = SPI_DATA_8BIT,
    .clock_phase_sel = SPI_SAMPLE_LEADING_EDGE, 
    .clock_polarity_sel = SPI_INACTIVE_LOW,  
    .clock_divider = 1 
};


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



// Function Prototypes 
void Timer_initModule();
void sendChar(char s);
uint32_t getCurrentTime(void); // Helper that gets system time



int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;   // Stop watchdog timer

    // Set P1.4 (Switch) as input with a pull-up resistor
    P1->DIR &= ~BIT4;
    P1->REN |= BIT4;
    P1->OUT |= BIT4;


    // SPI_TX LED
    P1->DIR |= BIT0;    // set BIT0 as OUTPUT
    P1->OUT &= ~(BIT0); // set BIT0 as LOW
    //clear secondary functions
    P1->SEL0 &= ~(BIT0);
    P1->SEL1 &= ~(BIT0);



    // Enable interrupt for P1.4
    P1->IES |= BIT4;   // Set interrupt edge select (falling edge)
    P1->IFG &= ~BIT4;  // Clear interrupt flag
    P1->IE |= BIT4;    // Enable interrupt


    UART_initModule(EUSCI_A0, &UART_A0_config); 
    UART_enableModule(EUSCI_A0); 

    // Enable UART0 Pins
    // P1.2->RX
    // P1.3->TX
    P1->SEL0 |= BIT2 | BIT3;
    P1->SEL1 &= ~(BIT2 | BIT3);


    SPI_initModule(EUSCI_B0, &B0_SPI_config);  // Initialize SPI communication
    SPI_enableModule(EUSCI_B0);
    EUSCI_B0->STATW |= EUSCI_B_STATW_LISTEN;   // Enable loopback mode


    // Enable Timer A0
    Timer_initModule();

    NVIC_EnableIRQ(PORT1_IRQn);  // Enable Port1 interrupt in the NVIC
    NVIC_EnableIRQ(TA0_0_IRQn);  // Enable Timer A0 interrupt in the NVIC
    __enable_irq();

    while (1)
    {

        // This part is left empty since the data transmission will be handled by the interrupt.
        // The main loop will be waiting for the button press and respond by sending the data.
        if (transmit_is_ready) { 
            
            prevTransmitTime = getCurrentTime();

            if(packetIndex >= PACKET_SIZE)  // Rollover to 0 if index exceeds size 
                packetIndex = 0; 


            /* SPI Loopback test */

            // Send Packet
            SPI_sendByte(EUSCI_B0, packet[packetIndex]);

            // Receive Packet just sent
            uint8_t byte = SPI_receiveByte(EUSCI_B0);
            sendChar(byte); // Echo the packet to UART 

            // Turn transmit LED on 
            P1->OUT |= BIT0;
            
            packetIndex++; 
            led_is_on = true;
            transmit_is_ready = false;
        }        
        else if (led_is_on && (getCurrentTime() - prevTransmitTime >= DELAY_1000_MS)) {
            //prevTransmitTime = getCurrentTime();
            // Toggle transmit LED off
            P1->OUT &= ~(BIT0);
            led_is_on = false; 
        }

    }
}



void PORT1_IRQHandler(void) {
    // Clear interrupt flag 
    if(P1->IFG & BIT4) { 
        P1->IFG  &= ~BIT4;
        transmit_is_ready = true; 
    }
}



// Triggers an interrupt every 1ms and increments a global counter
void TA0_0_IRQHandler(void){
    // Clear interrupt flag first
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    millis++;
}

void Timer_initModule() { 

  /* ================================ */
  // Timer_A0 configuration 
  // Generates an interrupt every 1ms
  /*==================================*/

  // Clear previous configurations
  TIMER_A0->CTL &= ~(TIMER_A_CTL_SSEL_MASK | TIMER_A_CTL_ID_MASK | TIMER_A_CTL_MC_MASK | TIMER_A_CTL_IFG) ;  
  TIMER_A0->CCTL[0] &= ~(TIMER_A_CCTLN_CM_MASK | TIMER_A_CCTLN_CAP | TIMER_A_CCTLN_OUTMOD_MASK | TIMER_A_CCTLN_CCIE | TIMER_A_CCTLN_CCIFG);
  
  // Set clock source to ACLK (32.768kHz)
  TIMER_A0->CTL |= TIMER_A_CTL_TASSEL_1;

  // Input Clock divider 
  TIMER_A0->CTL |= TIMER_A_CTL_ID__1;

  // Set Timer base period  
  TIMER_A0->CCR[0] = PERIOD_1000_HZ; 

  // Set Compare Mode and Toggle_Reset Mode
  TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_OUTMOD2;

  // Enable Interrupt 
  TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE;

  // Start counter in Up mode 
  TIMER_A0->CTL |= TIMER_A_CTL_MC__UP;

}


void sendChar(char s)
{
    // Echo character sent back to host PC serial monitor
    // Add the condition inside while loop
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG)); // wait till TXBUF is empty (Check TXIFG flag)
        EUSCI_A0->TXBUF = s; // send character through buffer

    //while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG)); // wait till TXBUF is empty (Check TXIFG flag)
    //EUSCI_A2->TXBUF = s; // send character through buffer
}


uint32_t getCurrentTime(void) { 
    return millis;
}