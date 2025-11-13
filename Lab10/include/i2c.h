
#ifndef I2C_H
#define I2C_H


#include "msp432p401r.h"

/// @brief Initializes the EUSCI B0 I2C module 
void I2C_initModule();

/// @brief Enables I2C interrupts for the specified I2C module
/// @param i2c Pointer to the EUSCI_B_Type I2C module
void I2C_enableInterrupts(EUSCI_B_Type* i2c);

#endif