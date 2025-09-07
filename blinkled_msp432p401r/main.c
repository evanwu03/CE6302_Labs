// This application reads the xyz data from the 3-axis KXTC9-2050 accelerometer
// located on boostxl_edumkii and prints them to the LCD screen. Additionally 
// the xyz values will be communicated over UART to a MATLAB script for visualization


#include <ti/devices/msp432p4xx/driverlib/driverlib.h>


#define ENABLE_ADC_REPEATMODE 1

static const uint8_t ACCEL_X_PIN = GPIO_PIN1;  // accelerometer X axis pin 
static const uint8_t ACCEL_Y_PIN = GPIO_PIN4;  // accelerometer Y axis pin
static const uint8_t ACCEL_Z_PIN = GPIO_PIN2;  // accelerometer Z axis pin 

// ADC results buffer for accelerometer
static uint16_t resultsBuffer[3];


// Periperhal Initialization Functions
void initializePeripherals(); 
void adc_init();
void gpio_init();
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

    while(1)
    {
        PCM_gotoLPM0();
    }
}




/// @brief Initializes all peripherals used for application
void initializePeripherals() {

// Configure pins for ADC input
gpio_init();


// Configure System clock 
system_clock_init();

// Enable ADC module 
adc_init();

}

void system_clock_init() {

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

}

void adc_init() { 

    // Peripheral clock gating for ADC
    //ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_1, ADC_DIVIDER_1, ADC_MAPINTCH0); 
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

void gpio_init() { 
    // Set GPIO pins as ADC input
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, ACCEL_Y_PIN | ACCEL_Z_PIN, GPIO_TERTIARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, ACCEL_X_PIN, GPIO_TERTIARY_MODULE_FUNCTION);
}

/// @brief Triggered awhenever conversion is completed and result is placed in 
/// ADC memory (to be defined). The results array is then grabbed and placed in a results buffer
/// @param  
void ADC14_IRQHandler(void) {


    // Check ADC interrupt sequence status 


    // Once ADC conversions are completed, store in buffer
}