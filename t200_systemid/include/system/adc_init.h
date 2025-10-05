
#ifndef ADC_INIT_H
#define ADC_INIT_H


// ADC Configurations
#define ENABLE_ADC_REPEATMODE 1


extern volatile uint8_t  data_is_ready;
extern volatile uint16_t rawCurrent;

// @brief Triggered awhenever conversion is completed and result is placed in
/// ADC memory (to be defined). The results array is then grabbed and placed in a results buffer
/// @param
void ADC14_IRQHandler(void);

/// @brief Initializes ADC14 Module and sets to single-sampling mode
void adc_init();




#endif //ADC_INIT_H