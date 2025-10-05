/* 
Purpose: Control a T200 thruster using a predefined square and sine wave of PWM values 
*/

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "printf.h"


// ADC Configurations
#define ENABLE_ADC_REPEATMODE 1


// Hall Effect Sensor Sampling pin 
static const uint8_t HALL_PIN = GPIO_PIN1; // P6.1 (ADC)


// UART pin definition 
static const uint8_t RX_PIN = GPIO_PIN2; //  P1.2 (RX)
static const uint8_t TX_PIN = GPIO_PIN3; //  P1.3 (TX)


// T200 ESC PWM Pin definition
static const uint8_t PWM_PIN = GPIO_PIN5; // P2.5


// Peripheral Initialization Functions
void initializePeripherals();
void adc_init();
void gpio_init();
void timer_init();
void uart_init();
void system_clock_init();


// Helper functions
uint_fast16_t convert_to_duty(const uint_fast16_t pwm_period_us, const uint_fast16_t ccr, const uint_fast16_t pulsewidth);


//  Interrupt Service Routines
void ADC14_IRQHandler(void);
void TA0_0_IRQHandler(void);


//![Simple UART Config]
/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 *http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
static const eUSCI_UART_ConfigV1 uartConfig =
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


/*
The PWM timer that drives the BlueRobotics T200 is configured to 400Hz according 
to maximum update rate given by the manufacturer's spec sheet. The T200 only accepts pulse width 
periods between 1100-1900us. 
*/

static const uint_fast16_t pwm_period_400Hz_us = 2500; 
static const uint_fast16_t pwm_period_400Hz_CCR = 29999;
static const uint_fast16_t stop_signal_us = 1500;


Timer_A_PWMConfig pwmConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        pwm_period_400Hz_CCR,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        (stop_signal_us/pwm_period_400Hz_us)*pwm_period_400Hz_CCR                           // By default T200 should be off
        
};


// The timer is configured to interrupt at 12.8kHz to update the duty cycle in
// sin_table at time steps of 78.125uS. 
// CCR0 = SMCLK frequency / target frequency -1
// CCR0 = 12Mhz / 12.8kHz - 1 = 937
static const uint16_t updatePeriod = 937;  
static const Timer_A_UpModeConfig updateTableConfig = 
{
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_1,
    updatePeriod, 
    TIMER_A_TAIE_INTERRUPT_DISABLE,     // Don't enable this one before it raises interrupt after counter resets to 0
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Raise interrupt to happen when counter == CCR0  
    TIMER_A_DO_CLEAR
};


// Raw current value from ADC (14bits: 0-16383)
static uint16_t rawCurrent;

// Global status flag
static volatile uint8_t data_is_ready; // Check if data from ADC has been received and is ready to send


// Index to table, Note: can we make this not a global variable?
static volatile uint8_t tbl_index = 0;


#define TABLE_SIZE 128
// 100 Hz sine lookup table of PWM periods
uint16_t sin_table[TABLE_SIZE] = {
1750, 1757, 1764, 1771, 1778, 1785, 1792, 1799,
1806, 1813, 1820, 1827, 1834, 1841, 1848, 1855,
1862, 1869, 1875, 1881, 1887, 1892, 1897, 1902,
1906, 1910, 1914, 1917, 1920, 1923, 1925, 1926,
1927, 1927, 1927, 1926, 1925, 1923, 1920, 1917,
1914, 1910, 1906, 1902, 1897, 1892, 1887, 1881,
1875, 1869, 1862, 1855, 1848, 1841, 1834, 1827,
1820, 1813, 1806, 1799, 1792, 1785, 1778, 1771,
1764, 1757, 1750, 1743, 1736, 1729, 1722, 1715,
1708, 1701, 1694, 1687, 1680, 1673, 1666, 1659,
1652, 1645, 1638, 1631, 1624, 1617, 1610, 1603,
1597, 1591, 1585, 1580, 1575, 1570, 1566, 1562,
1558, 1555, 1552, 1550, 1548, 1547, 1546, 1546,
1546, 1547, 1548, 1550, 1552, 1555, 1558, 1562,
1566, 1570, 1575, 1580, 1585, 1591, 1597, 1603,
1610, 1617, 1624, 1631, 1638, 1645, 1652, 1659
};



int main(void)
{
    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);
    // Disable all interrupts
    Interrupt_disableMaster();

    initializePeripherals();

    // Enable interrupts after initializing
    Interrupt_enableInterrupt(INT_ADC14);
    Interrupt_enableInterrupt(INT_TA0_0);

    //Interrupt_enableInterrupt(INT_EUSCIA0);
    Interrupt_enableMaster();

    while (1)
    {

        
        if (data_is_ready)
        {


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
    adc_init();

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

    ADC14_configureSingleSampleMode(ADC_MEM0, ENABLE_ADC_REPEATMODE);
    // Enable ADC module
    ADC14_enableModule();

    // Enable interrupt on ADC channel 2 (end of sequence)
    ADC14_enableInterrupt(ADC_INT0);

    // enables sample timer used to take samples
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);


    // Both ADC14_enableConversion and ADC14_toggleConversionTrigger
    // must be called to begin sampling
    ADC14_enableConversion();
    
    // Sets source of the ADC trigger. In this code, Timer A CCR1 will be used
    // which is set to interrupt every 1ms (1khz) on timer's rising edge.
    ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE1, false); 
}

/// @brief Initializes all GPIO Pins used in application
void gpio_init()
{
    // Set GPIO pins as ADC input
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, HALL_PIN, GPIO_TERTIARY_MODULE_FUNCTION);
    
    // Configure P3.2 (TX) and P3.3 (RX) as UART pins
    //GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, RX_PIN | TX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    // Configure P1.2 (RX) and P1.3 (TX)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            RX_PIN | TX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    
    // Configure P2.5 to generate PWM to T200 thruster
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, PWM_PIN,
                 GPIO_PRIMARY_MODULE_FUNCTION);
}



/// @brief Schedules Timer A to update current entry in the PWM table
void timer_init() { 

    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig);
    // This starts the timer that will update the duty cycle 
    Timer_A_configureUpMode(TIMER_A0_BASE, &updateTableConfig);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

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
    if (status & ADC_INT0)
    {
        // Once ADC conversions are completed, store in buffer
        // Make sure size of buffer matches the number of sequences
        rawCurrent = ADC14_getResult(ADC_MEM0);
    }

    // Set data_read flag, letting UART transfer initiate in main
    data_is_ready = true;
}


void TA0_0_IRQHandler(void)
{

    // Clear the interrupt flag and reset timer count
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    pwmConfig.dutyCycle = convert_to_duty(pwm_period_400Hz_us, pwm_period_400Hz_CCR, sin_table[tbl_index]);

    Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, pwmConfig.dutyCycle);
    
    tbl_index++;

    // Wrap around when end of table is reached.
    if (tbl_index >= TABLE_SIZE)
        tbl_index = 0;
}


/// @brief 
/// Helper function that converts desired pulsewidth (in microseconds) to 
/// duty cycle value as a percentage of Compare-Capture Register
/// @param pwm_period_us 
/// @param ccr 
/// @param pulsewidth_us 
uint_fast16_t convert_to_duty(const uint_fast16_t pwm_period_us, const uint_fast16_t ccr, const uint_fast16_t pulsewidth_us) {
    return (pulsewidth_us * ccr) / pwm_period_us;
}
