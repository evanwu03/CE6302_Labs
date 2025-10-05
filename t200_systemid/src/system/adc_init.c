
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "../../include/system/adc_init.h"


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

