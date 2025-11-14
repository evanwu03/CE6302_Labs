
#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include <stdint.h>
#include <stddef.h>



/// @brief GPIO port structure
struct gpio { 
    uintptr_t base;    
    size_t pin; // pin mask BITO, BIT1, ...
    uint8_t pull_en; // pull-up/pull-down enable
};



/// @brief Initialize a GPIO port as output
/// @param gpio_port GPIO Port
/// @param base Base address of the GPIO port
/// @param pinMask Pin mask for the desired pin(s)
void gpio_init_output(struct gpio* gpio_port, unsigned long base, size_t pinMask);


/// @brief Initialize a GPIO port as output
/// @param gpio_port GPIO Port
/// @param base Base address of the GPIO port
/// @param pinMask Pin mask for the desired pin(s)
void gpio_init_input(struct gpio* gpio_port, unsigned long base, size_t pinMask, uint8_t pull_en);


/// @brief Write a value to the GPIO port
/// @param gpio_port GPIO Port
/// @param value Value to write (true for high, false for low)
void gpio_write(struct gpio* gpio_port, _Bool value);


/// @brief Toggle the state of the GPIO port
/// @param gpio_port GPIO Port
void gpio_toggle(struct gpio* gpio_port);


/// @brief Read the current state of the GPIO port
/// @param gpio_port GPIO Port
/// @return Current state of the GPIO port (true for high, false for low)
_Bool gpio_read(struct gpio* gpio_port);  



#endif