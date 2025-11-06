


#ifndef TIMER_INIT_H
#define TIMER_INIT_H



/*
The PWM timer that drives the BlueRobotics T200 is configured to 400Hz according 
to maximum update rate given by the manufacturer's spec sheet. The T200 only accepts pulse width 
periods between 1100-1900us. 
*/


extern volatile uint8_t tbl_index; // tbl_index 
#define RINGBUF_SIZE (2048U)
extern volatile uint16_t ringbuf[]; 
extern volatile uint16_t head; 
extern volatile uint16_t tail;


extern Timer_A_PWMConfig pwmConfig;

/* Function Prototypes*/

void TA0_0_IRQHandler(void);


/// @brief Schedules Timer A to update current entry in the PWM table
void timer_init();


/// @brief 
/// Helper function that converts desired pulsewidth (in microseconds) to 
/// duty cycle value as a percentage of Compare-Capture Register
/// @param pwm_period_us 
/// @param ccr 
/// @param pulsewidth_us 
uint_fast16_t convert_to_duty(const uint_fast16_t pwm_period_us, const uint_fast16_t ccr, const uint_fast16_t pulsewidth);

#endif // TIMER_INIT_H
