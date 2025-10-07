

#ifndef GPIO_INIT_H
#define GPIO_INIT_H


// Hall Effect Sensor Sampling pin 
#define HALL_PIN GPIO_PIN1 // P6.1 (ADC)

// UART pin definition 
#define RX_PIN (GPIO_PIN2)  //  P1.2 (RX)
#define TX_PIN (GPIO_PIN3)  //  P1.3 (TX)


// T200 ESC PWM Pin definition
#define PWM_PIN (GPIO_PIN4)

// Debug Pin
#define DEBUG_PIN (GPIO_PIN1)

// GPIO initialization
void gpio_init();



#endif // GPIO_INIT_H
