

// SLAVE RX ADDED LED TO SEE RECEIVED DATA 
#include "msp.h"
#include <stdint.h>
#include <stdbool.h>

// --- Copied from Master ---
// Include the path to your uart.h file.
// Make sure this path is correct for your slave project structure.
#include "../include/uart.h" 

// UART Configuration struct copied from master
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
// --- End of Copied Section ---


// Delay period 
#define PERIOD_1000_HZ 33 
#define DELAY_1000_MS  1000

volatile uint8_t receivedData;
volatile bool rx_ready = false; 

// RX Led flag state
bool led_is_on = false; 

// Global counter to keep track of time
volatile uint32_t millis = 0;
uint32_t prevTransmitTime = 0;

// Function Prototype 
void printReceivedData(); 
void sendChar(char s);
uint32_t getCurrentTime(void); // Helper that gets system time
void Timer_initModule();
void initSPI();



int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // Stop watchdog timer

    // --- Copied from Master ---
    // Initialize and enable UART EUSCI_A0
    UART_initModule(EUSCI_A0, &UART_A0_config); 
    UART_enableModule(EUSCI_A0); 

    // Enable UART0 Pins
    // P1.2->RX
    // P1.3->TX
    P1->SEL0 |= BIT2 | BIT3;
    P1->SEL1 &= ~(BIT2 | BIT3);
    // --- End of Copied Section ---


    // SPI_RX LED
    P2->DIR |= BIT1;    // set P2.1 (LED1 on lab manual) as OUTPUT
    P2->OUT &= ~(BIT1);
    
    //clear secondary functions
    P2->SEL0 &= ~(BIT1);
    P2->SEL1 &= ~(BIT1);


    
    // Initialize Timer A0 
    Timer_initModule();
    
    // Initialize SPI communication
    initSPI();
    
    NVIC_EnableIRQ(EUSCIB0_IRQn); // Enable SPI interrupt in the NVIC
    NVIC_EnableIRQ(TA0_0_IRQn);  // Enable Timer A0 interrupt in the NVIC
    __enable_interrupt(); // Enable global interrupts

    while (1)
    {
        // This part is left empty since the received data will be handled by the interrupt.
        if(rx_ready) { 

            prevTransmitTime = getCurrentTime();

            printReceivedData();


            // Turn on RX LED
            //P1->OUT |= BIT0;
            P2->OUT |= BIT1;
            led_is_on = true;
            rx_ready = false; 
        }
        else if (led_is_on && (getCurrentTime() - prevTransmitTime >= DELAY_1000_MS)) { 
            // Toggle RX LED off
            //P1->OUT &= ~(BIT0);
            P2->OUT &= ~BIT1;

            led_is_on = false; 
        }
    }
}


void EUSCIB0_IRQHandler(void)
{
    // Check if the Receive Interrupt Flag is set
    if (EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG)
    {
        // Data has been received
        receivedData = EUSCI_B0->RXBUF; // Read data from buffer to clear flag
        // Toggle LED1 (P2.1) to indicate successful reception [cite: 76]
        // P2->OUT ^= BIT1;
        
        rx_ready = true; 

        // Call function to print the data
        //printReceivedData();
    }
}

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


void printReceivedData()
{
    // This function is called from the ISR to print the received data
    // to the console, as required by the lab manual. [cite: 17, 79]
    sendChar(receivedData);
}


// --- Copied from Master ---
// sendChar function to transmit a single character via UART
void sendChar(char s)
{
    // wait till TXBUF is empty (Check TXIFG flag)
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
    
    EUSCI_A0->TXBUF = s; // send character through buffer
}
// --- End of Copied Section ---


uint32_t getCurrentTime(void) { 
    return millis;
}


void initSPI()
{
    // Configure the SPI pins
    P1->SEL0 |= BIT5 | BIT6 | BIT7; // Set P1.5, P1.6, and P1.7 as SPI pins (CLK, MOSI, MISO)
    P1->SEL1 &= ~(BIT5 | BIT6 | BIT7);

    // Configure the SPI slave mode
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST; // Put eUSCI state machine in reset

    /*
     * Configure the EUSCI_B0_CTLW0 register:
     * 1. Set UCSYNC = 1 for Synchronous SPI mode
     * 2. All other settings for Mode 0 Slave (3-pin, LSB first) are defaults (0):
     * - UCMST = 0 (Slave mode - default)
     * - UCMSB = 0 (LSB first) (Matches your master config)
     * - UC7BIT = 0 (8-bit data)
     * - UCCKPL = 0 (Clock Polarity, idle low - Mode 0)
     * - UCCKPH = 0 (Clock Phase, sample on 1st edge - Mode 0)
     * - UCMODEx = 00 (3-pin SPI)
     */
    EUSCI_B0->CTLW0 = EUSCI_B_CTLW0_SWRST | EUSCI_B_CTLW0_SYNC;

    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST; // Release eUSCI state machine from reset

    
    // Enable SPI receive interrupt
    EUSCI_B0->IFG &= ~EUSCI_B_IFG_RXIFG;
    EUSCI_B0->IE |= EUSCI_B_IE_RXIE; // Enable RX interrupt
}


