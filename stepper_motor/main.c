/* 
Purpose: Control a Servo motor using a predefined square and sine wave of PWM values 
*/

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "printf.h"


// ADC Configurations
#define ENABLE_ADC_REPEATMODE 1

// Accelerometer pin definition
//static const uint8_t ACCEL_X_PIN = GPIO_PIN1; // accelerometer X axis pin
//static const uint8_t ACCEL_Y_PIN = GPIO_PIN4; // accelerometer Y axis pin
//static const uint8_t ACCEL_Z_PIN = GPIO_PIN2; // accelerometer Z axis pin


// UART pin definition 
static const uint8_t RX_PIN = GPIO_PIN2; //  P1.2 (RX)
static const uint8_t TX_PIN = GPIO_PIN3; //  P1.3 (TX)


// Stepper Motor Pin definitions
static const uint8_t SERVO_PIN = GPIO_PIN5; // P2.5



//![Simple UART Config]
/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 *http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const eUSCI_UART_ConfigV1 uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        78,                                     // BRDIV = 78
        2,                                       // UCxBRF = 2
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
        EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
};
 
Timer_A_PWMConfig pwmConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        32000,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        3200
};

// ADC results buffer for accelerometer
static uint16_t resultsBuffer[3];

// Global status flag
static volatile uint8_t data_is_ready; // Check if data from ADC has been received and is ready to send

// Periperhal Initialization Functions
void initializePeripherals();
void adc_init();
void gpio_init();
void timer_init();
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
    //Interrupt_enableInterrupt(INT_EUSCIA0);
    Interrupt_enableMaster();

    while (1)
    {


        //UART_transmitData(EUSCI_A0_BASE, 'f'); // Send f as a test
        // For some reason the ADC interferes with my UART polling
        
        
        if (data_is_ready)
        {
            int xdata = resultsBuffer[0];
            int ydata = resultsBuffer[1];
            int zdata = resultsBuffer[2];

            // Print XYZ Data
            //printf(EUSCI_A0_BASE, "X: %u\n", resultsBuffer[0]);
            //printf(EUSCI_A0_BASE, "Y: %u\n", resultsBuffer[1]);
            //printf(EUSCI_A0_BASE, "Z: %u\n", resultsBuffer[2]);

            printf(EUSCI_A0_BASE, "%u, %u, %u\n", xdata, ydata, zdata);
            
            
            // To do: Print LCD screen

            data_is_ready = false; // reset flag
        }
        
        PCM_gotoLPM0InterruptSafe(); // Go back to sleep
    }
}

/// @brief Initializes all peripherals used for application
void initializePeripherals()
{
    // Configure System clock
    system_clock_init();

    // Configure pins for ADC input
    gpio_init();



    // Enable ADC module
    //adc_init();

    // Enable Timer A 
    timer_init();

    // Enable UART module
    uart_init();
}

/// @brief Handles System Clock Configurations
void system_clock_init()
{

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
}

/// @brief Initializes ADC14 Module and sets multi-sequencing mode
void adc_init() {



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
    //GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, ACCEL_Y_PIN | ACCEL_Z_PIN, GPIO_TERTIARY_MODULE_FUNCTION);
    //GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, ACCEL_X_PIN, GPIO_TERTIARY_MODULE_FUNCTION);
    
    // Configure P3.2 (TX) and P3.3 (RX) as UART pins
    //GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, RX_PIN | TX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    // Configure P1.2 (RX) and P1.3 (TX)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            RX_PIN | TX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    
    // Configure P2.4 to generate PWM to servo motor
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, SERVO_PIN,
                 GPIO_PRIMARY_MODULE_FUNCTION);
}



/// @brief Initializes Timer A for PWM output to servo
void timer_init() { 

    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

}


/// @brief Initializes UART peripheral
void uart_init()
{


    // Initialize UART module A0 with local config
    UART_initModule(EUSCI_A0_BASE, &uartConfig);

    // Enable UART module
    UART_enableModule(EUSCI_A0_BASE);

    //UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);

}

/// @brief Triggered awhenever conversion is completed and result is placed in
/// ADC memory (to be defined). The results array is then grabbed and placed in a results buffer
/// @param
void ADC14_IRQHandler(void)
{
    uint64_t status = ADC14_getEnabledInterruptStatus();

    // Clear the interrupt flag
    ADC14_clearInterruptFlag(status);

    // Check ADC interrupt sequence status
    if (status & ADC_INT2)
    {
        // Once ADC conversions are completed, store in buffer
        // Make sure size of buffer matches the number of sequences
        ADC14_getMultiSequenceResult(resultsBuffer);
    }

    // Set data_read flag, letting UART transfer initiate in main
    data_is_ready = true;
}
