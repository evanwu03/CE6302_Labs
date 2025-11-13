
#include <stdint.h>
#include <stdbool.h>
#include "../include/uart.h"
#include "../include/spi.h"
#include "../include/i2c.h"
#include <msp432p401r.h>


#define PACKET_SIZE (4U)
char Packet[]={0x03,0xAA,0xBB,0xCC};
volatile int Data_Cnt=0;


void main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // Stop the WatchDog Timer
 

    // Set SDA and SCL
    P1->SEL0 |= BIT6 | BIT7;    // P1.6 SDA
    P1->SEL1 &= ~(BIT6 | BIT7); // P1.7 SCL

    //Setup EUSCI_B0 as Master, write/transmitter
    I2C_initModule();

    I2C_enableInterrupts(EUSCI_B0);


    NVIC_EnableIRQ(EUSCIB0_IRQn);

   __enable_irq(); // enable global interrupt

    while(1){

        
        //while(EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);

        int i=0;
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;     // 1. Generate START condition
        
        for(i=0;i<100;i++){ // Delay

        } 
    }
}


// --ISR
void EUSCIB0_IRQHandler(void) {


    // Reset packet index 
    if (Data_Cnt >= PACKET_SIZE) { 
            Data_Cnt = 0;
    }


    if (EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) { 

    }
    EUSCI_B0->TXBUF = Packet[Data_Cnt];
    Data_Cnt++; 
    //EUSCI_B0->TXBUF = 0xAA;



}

