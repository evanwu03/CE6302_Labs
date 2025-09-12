// This application reads the xyz data from the 3-axis KXTC9-2050 accelerometer
// located on boostxl_edumkii and prints them to the LCD screen. Additionally
// the xyz values will be communicated over UART to a MATLAB script for visualization

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#define ENABLE_ADC_REPEATMODE 1


// Pin definition 
static const uint8_t ACCEL_X_PIN = GPIO_PIN1; // accelerometer X axis pin
static const uint8_t ACCEL_Y_PIN = GPIO_PIN4; // accelerometer Y axis pin
static const uint8_t ACCEL_Z_PIN = GPIO_PIN2; // accelerometer Z axis pin


static const uint8_t RX_PIN = GPIO_PIN2; 
static const uint8_t TX_PIN = GPIO_PIN3; 

// ADC results buffer for accelerometer
static uint16_t resultsBuffer[3];

// Global status flag
static uint8_t data_is_ready; // Check if data from ADC has been received and is ready to send

// Periperhal Initialization Functions
void initializePeripherals();
void adc_init();
void gpio_init();
void uart_init();
void system_clock_init();

// ADC Interrupt Handler
void ADC14_IRQHandler(void);

int main(void)
{
    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);
    // Disable all interrupts
    Interrupt_disableMaster();

    initializePeripherals();

    // Enable interrupts after initializing
    Interrupt_enableInterrupt(INT_ADC14);
    Interrupt_enableMaster();

    while (1)
    {
        if (data_is_ready)
        {

            // XYZ Data
            transmitData(XYZ);

            // Print LCD screen
        }
        PCM_gotoLPM0(); // Go back to sleep
    }
}

/// @brief Initializes all peripherals used for application
void initializePeripherals()
{

    // Configure pins for ADC input
    gpio_init();

    // Configure System clock
    system_clock_init();

    // Enable ADC module
    adc_init();
}

/// @brief Handles System Clock Configurations
void system_clock_init()
{

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
}

/// @brief Initializes ADC14 Module and sets multi-sequencing mode
void adc_init()
{

    // Peripheral clock gating for ADC // check this sampling rate
    ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, ADC_NOROUTE);

    // Configure for multi-sequence mode since we are
    // Sampling from 3 ADC inputs of accelerometer at once
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, ENABLE_ADC_REPEATMODE);

    // Configure memory location for samples to be stored
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, ADC_NONDIFFERENTIAL_INPUTS);
    ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);
    ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A11, ADC_NONDIFFERENTIAL_INPUTS);

    // Enable ADC module
    ADC14_enableModule();

    // Enable interrupt on ADC channel 2 (end of sequence)
    ADC14_enableInterrupt(ADC_INT2);

    // enables sample timer used to take samples
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    // Both ADC14_eanbleConversion and ADC14_toggleConversionTrigger
    // must be called to begin sampling
    ADC14_enableConversion();
    ADC14_toggleConversionTrigger();
}

/// @brief Initializes all GPIO Pins used in application
void gpio_init()
{
    // Set GPIO pins as ADC input
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, ACCEL_Y_PIN | ACCEL_Z_PIN, GPIO_TERTIARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, ACCEL_X_PIN, GPIO_TERTIARY_MODULE_FUNCTION);
}

/// @brief Initializes UART peripheral
void uart_init()
{
    // Configure P3.2 (TX) and P3.3 (RX) as UART pins
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, RX_PIN | TX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    // Local UART configuration (9600 baud, 8N1, SMCLK @ 48 MHz)
    const eUSCI_UART_Config uartConfig =
        {
            EUSCI_A_UART_CLOCKSOURCE_SMCLK,               // Clock source = SMCLK (48 MHz)
            312,                                          // BRDIV = 312
            8,                                            // UCxBRF = 8
            0,                                            // UCxBRS = 0
            EUSCI_A_UART_NO_PARITY,                       // No Parity
            EUSCI_A_UART_LSB_FIRST,                       // LSB First
            EUSCI_A_UART_ONE_STOP_BIT,                    // One stop bit
            EUSCI_A_UART_MODE,                            // UART mode
            EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION // Oversampling
        };

    // Initialize UART module A2 with local config
    UART_initModule(EUSCI_A2_BASE, &uartConfig);

    // Enable UART module
    UART_enableModule(EUSCI_A2_BASE);
}

/// @brief Triggered awhenever conversion is completed and result is placed in
/// ADC memory (to be defined). The results array is then grabbed and placed in a results buffer
/// @param
void ADC14_IRQHandler(void)
{
    uint64_t status = ADC14_getEnabledInterruptStatus();

    // Clear the interrupt flag
    ADC14_clearInterruptFlag(ADC_INT2);

    // Check ADC interrupt sequence status
    if (status & ADC_INT2)
    {

        // Once ADC conversions are completed, store in buffer
        // Make sure size of buffer matches the number of sequences
        ADC14_getMultiSequenceResult(&resultsBuffer);
    }

    // Set data_read flag, letting UART transfer initiate in main
    data_is_ready = true;
}

void transmit_data(const int* accelData) { 

    

}