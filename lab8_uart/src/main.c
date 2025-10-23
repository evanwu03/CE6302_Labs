

#include <stdint.h>
//#include <msp432.h>
#include <msp432p401r.h>
#include <stdbool.h>


/* =========================================== */
// UART_Config_t configuration parameter types
/* =========================================== */

typedef enum {
    UART_PARITY_NONE = 0,
    UART_PARITY_ODD = EUSCI_A_CTLW0_PEN,
    UART_PARITY_EVEN = EUSCI_A_CTLW0_PAR | EUSCI_A_CTLW0_PEN
} UART_Parity_t;

typedef enum {
    UART_LSB_FIRST = 0,
    UART_MSB_FIRST = EUSCI_A_CTLW0_MSB
} UART_DataOrder_t;

typedef enum {
    UART_DATA_8BIT = 0,
    UART_DATA_7BIT = EUSCI_A_CTLW0_SEVENBIT
} UART_Character_Len_t;

typedef enum {
    UART_MODE = EUSCI_A_CTLW0_MODE_0,
    UART_IDLE_LINE_MULTI = EUSCI_A_CTLW0_MODE_1,
    UART_ADDRESS_BIT_MULTI = EUSCI_A_CTLW0_MODE_2,
    UART_AUTO_BAUD_DETECT = EUSCI_A_CTLW0_MODE_3
} UART_Mode_t;

typedef enum {
    UART_UCLK = EUSCI_A_CTLW0_SSEL__UCLK,
    UART_ACLK = EUSCI_A_CTLW0_SSEL__ACLK,
    UART_SMCLK = EUSCI_A_CTLW0_SSEL__SMCLK,
} UART_Clock_Source_t;

// UCAxMCTLW Register configs

typedef enum {
    UART_OVERSAMPLING_OFF = 0,
    UART_OVERSAMPLING_ON = EUSCI_A_MCTLW_OS16
} UART_Oversampling_t;

// End of UART parameter definitions

/* =========================================== */
// UART_Config_t Definition
/* =========================================== */
typedef struct {
    UART_Parity_t parity;             // Parity Enable
    UART_DataOrder_t order;           // MSB mode select
    UART_Character_Len_t data_length; // Character length
    UART_Mode_t mode;                 // eUSCI_A mode select
    UART_Clock_Source_t clock_sel;    // eUSCI_A clock source select
    uint32_t baud_rate;               // Desired baud rate
    UART_Oversampling_t oversampling; // Enables oversmapling mode;      UCAxMCTLW:  UCOS16 bit
    uint16_t baud_prescaler;          // baud rate prescaler select;     UCAxBRW:    UCBRx field
    uint8_t firstMod;                 // first modualtion stage select;  UCAxMCTLW:  UCBRFx field
    uint8_t secondMod;                // second modulation stage select; UCAxMCTLW:  UCBRSx field

} UART_config_t;

// End of UART_Config_t definition



// Clock Frequency Defines
#define ACLK_FREQ_HZ (32768U)
#define SMCLK_FREQ_HZ (1000000U)


// Function prototypes 
// UART drivers
void UART_initModule(EUSCI_A_Type *uart, const UART_config_t *config);
void UART_enableModule(EUSCI_A_Type *uart);
void UART_disableModule(EUSCI_A_Type *uart);
void UART_enableInterrupts(EUSCI_A_Type *uart, uint8_t mask);
void UART_disableInterrupts(EUSCI_A_Type *uart, uint8_t mask);

// UART transmit functions
void sendString(char *str);
void sendChar(char s);


// UART configurations 
// Clock frequency:  1MHz (SMCLK)
// Baud rate: 9600 


static const UART_config_t UART_A0_config = {
    .parity = UART_PARITY_NONE, 
    .order  = UART_DATA_8BIT, 
    .data_length = UART_LSB_FIRST, 
    .mode = UART_MODE, 
    .clock_sel = UART_SMCLK, 
    .baud_rate = 9600, 
    .oversampling = UART_OVERSAMPLING_ON,
    .baud_prescaler = 6,
    .firstMod  = 8,
    .secondMod = 0x20
};


static const UART_config_t UART_A2_config = {
    .parity = UART_PARITY_NONE, 
    .order  = UART_DATA_8BIT, 
    .data_length = UART_LSB_FIRST, 
    .mode = UART_MODE, 
    .clock_sel = UART_SMCLK, 
    .baud_rate = 9600, 
    .oversampling = UART_OVERSAMPLING_ON,
    .baud_prescaler = 6,
    .firstMod  = 8,
    .secondMod = 0x20
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
    // EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST;                               // Clear previous configuration of UART

    // Add the configuration setup code here
    UART_initModule(EUSCI_A0, &UART_A0_config); 
    UART_enableModule(EUSCI_A0); 

    // Enable UART2 Pins
    // P3.2->RX
    // P3.3->TX
    P3->SEL0 |= BIT2 | BIT3;
    P3->SEL1 &= ~(BIT2 | BIT3);

    // UART2 Configuration
    // EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST;                 // Clear previous configuration of UART by setting reset


    
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

        while (EUSCI_A0->IFG & EUSCI_A_IFG_TXCPTIFG); // wait till TXBUF is empty (Check TXIFG flag)
        EUSCI_A0->TXBUF = str[i]; // send character through buffer
        i++;
    }
}

void sendChar(char s)
{

    // Add the condition inside while loop

    while (EUSCI_A2->IFG & EUSCI_A_IFG_TXCPTIFG); // wait till TXBUF is empty (Check TXIFG flag)
   
    EUSCI_A2->TXBUF = s; // send character through buffer
}




/// @brief HAL function for configuring EUSCI_Ax module for UART mode
/// @param uart THe UART module to be configured (A0, A1, or A2)
/// @param config A structure of type uart_config_t containing configuration parameters
void UART_initModule(EUSCI_A_Type *uart, const UART_config_t *config) {

    // Resets any previous UART configurations
    uart->CTLW0 = EUSCI_A_CTLW0_SWRST;

    // Select UART clock
    uart->CTLW0 |= config->clock_sel;

    uart->CTLW0 |= (config->parity | config->order | config->data_length | config->mode);

    // Set the baud rate generator prescale value
    uart->BRW = config->baud_prescaler;

    // Select first and second modulation stage values
    uart->MCTLW = 0;
    uart->MCTLW |= (config->firstMod << EUSCI_A_MCTLW_BRF_OFS) | (config->secondMod << EUSCI_A_MCTLW_BRS_OFS);

    // Set oversampling mode if enabled
    if (config->oversampling)
    {
        uart->MCTLW |= config->oversampling;
    }
}

/// @brief Enables the UART module by clearing the UCSWRST bit
/// @param uart // UART module
void UART_enableModule(EUSCI_A_Type *uart) {
    // Maybe check if UART is valid
    uart->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
}

/// @brief Disables the UART module by setting the UCSWRST bit
/// @param uart
void UART_disableModule(EUSCI_A_Type *uart) {
    uart->CTLW0 |= EUSCI_A_CTLW0_SWRST;
}

/// @brief Enables interrupt for selected UART module
/// @param uart
void UART_enableInterrupts(EUSCI_A_Type *uart, uint8_t mask) {

    uint8_t locMask;

    // Check for valid bits only
    locMask = (mask & (EUSCI_A_IE_RXIE | EUSCI_A_IE_TXIE | EUSCI_A_IE_STTIE | EUSCI_A_IE_TXCPTIE));

    uart->IE |= locMask;
}

/// @brief Disables interrupt for selected UART module
/// @param uart
void UART_disableInterrupts(EUSCI_A_Type *uart, uint8_t mask) {

    uint8_t locMask;

    // Check for valid bits only
    locMask = (mask & (EUSCI_A_IE_RXIE | EUSCI_A_IE_TXIE | EUSCI_A_IE_STTIE | EUSCI_A_IE_TXCPTIE));

    uart->IE &= ~locMask;
}