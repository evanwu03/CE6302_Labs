
#include <stdint.h>
#include <stdbool.h>
#include "../include/uart.h"
#include "../include/spi.h"
#include <msp432p401r.h>


#define PACKET_SIZE (4U)
char Packet[]={0x03,0xAA,0xBB,0xCC};
volatile int Data_Cnt=0;

// Function prototypes
void I2C_initModule();
void I2C_enableInterrupts(EUSCI_B_Type* i2c);


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


void I2C_initModule() {

    // Add the configuration code here..
    EUSCI_B0->CTLW0  =  EUSCI_B_CTLW0_SWRST; // Clear previous configurations 
    EUSCI_B0->CTLW0 |=  EUSCI_B_CTLW0_SYNC;   // Synchronous  

    // Set clock source
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SSEL__SMCLK; 

    // I2C mode 
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MODE_3;

    // Clock divider
    EUSCI_B0->BRW = 60; // BRW  
 
    // Configure to Master Mode 
    EUSCI_B0->CTLW0 |= (EUSCI_B_CTLW0_MST);

    // Transmit Mode
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR;

    // Set Slave address (of the PCF8523)

    //EUSCI_B0->I2CSA = 0b1101000; // 0xD8
    EUSCI_B0->I2CSA = 0x68; // 0xD8
    EUSCI_B0->CTLW0 &= ~(EUSCI_B_CTLW0_SLA10); // Address slave with 7 bits 

    // Set automatic STOP condition gneeration
    EUSCI_B0->CTLW1 |= EUSCI_B_CTLW1_ASTP_2;

    // Set Byte Counter threshold 
    //EUSCI_B0->TBCNT = 1; // Generate STOP condition after 1 byte has been sent/read
    EUSCI_B0->TBCNT = 4; // Generate STOP condition after 4 byte has been sent/read

    // Clear Reset bit
    EUSCI_B0->CTLW0  &=  ~(EUSCI_B_CTLW0_SWRST); // Clear previous configurations 

}


void I2C_enableInterrupts(EUSCI_B_Type* i2c) {
    i2c->IE |= EUSCI_B_IE_TXIE0; // Enable TX interrupt   
}