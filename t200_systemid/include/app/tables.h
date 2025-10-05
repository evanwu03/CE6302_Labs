
#ifndef TABLES_H
#define TABLES_H


#include <stdint.h>

#define TABLE_SIZE 128
// 100 Hz sine lookup table of PWM periods

extern const uint16_t sin_table[TABLE_SIZE];




#endif // TABLES_H