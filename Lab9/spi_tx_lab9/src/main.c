// MASTER TX ADDED LED TO SEE TRANSFER

#include <stdint.h>
#include <stdbool.h>
#include "../include/spi.h"
#include <msp432p401r.h>


#define PACKET_SIZE 4
uint8_t packet[PACKET_SIZE] = {0x7D, 0xAA, 0x33, 0x77};


volatile int packetIndex = 0;
volatile uint8_t transmit_is_ready = false; 
bool led_is_on = false; 

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


int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;   // Stop watchdog timer

    // Set P1.4 (Switch) as input with a pull-up resistor
    P1->DIR &= ~BIT4;
    P1->REN |= BIT4;
    P1->OUT |= BIT4;


    // Enable interrupt for P1.4
    P1->IES |= BIT4;   // Set interrupt edge select (falling edge)
    P1->IFG &= ~BIT4;  // Clear interrupt flag
    P1->IE |= BIT4;    // Enable interrupt


    SPI_initModule(EUSCI_B0, &B0_SPI_config);  // Initialize SPI communication
    SPI_enableModule(EUSCI_B0);

    NVIC_EnableIRQ(PORT1_IRQn);  // Enable Port1 interrupt in the NVIC

    while (1)
    {

        // SPI Loopback test 

        // This part is left empty since the data transmission will be handled by the interrupt.
        // The main loop will be waiting for the button press and respond by sending the data.
        if (transmit_is_ready) { 
            
            // lastTransmit = currentTime

            if(packetIndex >= PACKET_SIZE)  // Rollover to 0 if index exceeds size 
                packetIndex = 0; 

            
            // Send Packet
            SPI_sendByte(EUSCI_B0, packet[packetIndex]);
            // Turn transmit LED on 
            // led_is_on = true
            packetIndex++; 
            transmit_is_ready = false;
        }

        /*
        else if (led_is_on && (currentTime - lastTransmit >= 1000)) 

            previousTime = currentTime
            // Toggle transmit LED off
            led_is_on = false; 

        )
        */

    }
}



void PORT1_IRQHandler(void) {

    // Clear interrupt flag 


    transmit_is_ready = true; 
}

