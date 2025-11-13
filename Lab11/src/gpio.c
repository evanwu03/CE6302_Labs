


#include "../include/gpio.h"    
#include <msp432p401r.h>



void gpio_init_output(struct gpio* gpio_port, unsigned long base, size_t pinMask) { 
    gpio_port->base = base;
    gpio_port->pin = pinMask;
    gpio_port->pull_en = 0; // No pull resistor for output


    if (gpio_port_is_odd(gpio_port->base)) {
        DIO_PORT_Odd_Interruptable_Type* regs_odd = (DIO_PORT_Odd_Interruptable_Type*)gpio_port->base;
        regs_odd->DIR |= pinMask; // Set as output
        regs_odd->REN &= ~pinMask; // Disable pull resistors
    
    } else {
        DIO_PORT_Even_Interruptable_Type* regs_even = (DIO_PORT_Even_Interruptable_Type*)gpio_port->base;
        regs_even->DIR |= pinMask; // Set as output
        regs_even->REN &= ~pinMask; // Disable pull resistors
    }
    
}



void gpio_init_input(struct gpio *gpio_port, unsigned long base, size_t pinMask, uint8_t pull_en)
{

    gpio_port->base = base;
    gpio_port->pin = pinMask;
    gpio_port->pull_en = pull_en;

    if (gpio_port_is_odd(gpio_port->base))
    {
        DIO_PORT_Odd_Interruptable_Type *regs_odd = (DIO_PORT_Odd_Interruptable_Type *)gpio_port->base;
        if (pull_en) {
            regs_odd->REN |= pinMask; // Enable pull resistors
        }
        else {
            regs_odd->REN &= ~pinMask; // Disable pull resistors
        }

        regs_odd->DIR &= ~pinMask; // Set as input
    }
    else
    {
        DIO_PORT_Even_Interruptable_Type *regs_even = (DIO_PORT_Even_Interruptable_Type *)gpio_port->base;

        if (pull_en) {
            regs_even->REN |= pinMask; // Enable pull resistors
        }
        else {
            regs_even->REN &= ~pinMask; // Disable pull resistors
        }

        regs_even->DIR &= ~pinMask; // Set as input
    }

}



void gpio_write(struct gpio *gpio_port, _Bool value)
{

    if (gpio_port_is_odd(gpio_port->base))
    {
        DIO_PORT_Odd_Interruptable_Type *regs_odd = (DIO_PORT_Odd_Interruptable_Type *)gpio_port->base;
        if (value) {
            regs_odd->OUT |= gpio_port->pin;
        }
        else {
            regs_odd->OUT &= ~gpio_port->pin;
        }
    }
    else
    {
        DIO_PORT_Even_Interruptable_Type *regs_even = (DIO_PORT_Even_Interruptable_Type *)gpio_port->base;

        if (value) {
            regs_even->OUT |= gpio_port->pin;
        }
        else {
            regs_even->OUT &= ~gpio_port->pin;
        }
    }
}



void gpio_toggle(struct gpio* gpio_port) { 


    if (gpio_port_is_odd(gpio_port->base)) {
        DIO_PORT_Odd_Interruptable_Type* regs_odd = (DIO_PORT_Odd_Interruptable_Type*)gpio_port->base;
        regs_odd->OUT ^= gpio_port->pin;

    
    } else {
        DIO_PORT_Even_Interruptable_Type* regs_even = (DIO_PORT_Even_Interruptable_Type*)gpio_port->base;
        regs_even->OUT ^= gpio_port->pin;

    }
}



_Bool gpio_read(struct gpio* gpio_port) { 

    if (gpio_port_is_odd(gpio_port->base)) {
        DIO_PORT_Odd_Interruptable_Type* regs_odd = (DIO_PORT_Odd_Interruptable_Type*)gpio_port->base;
        return (regs_odd->IN & gpio_port->pin);

    
    } else {
        DIO_PORT_Even_Interruptable_Type* regs_even = (DIO_PORT_Even_Interruptable_Type*)gpio_port->base;
        return (regs_even->IN & gpio_port->pin);

    }

}


_Bool gpio_port_is_odd(uintptr_t port_addr)
{
    uint32_t block = (port_addr >> 8) & 0xFF;

    /* I really have no clue why ports are like this*/
    // P1 → 0x4C → even? No → odd port
    // P2 → 0x4D → odd  → even port
    // P3 → 0x4E → even → odd port
    // P4 → 0x4F → odd → even port

    return (block % 2 == 0);   // even block values = odd-numbered port
}

