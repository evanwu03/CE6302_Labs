

#include "../include/i2c.h"


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
