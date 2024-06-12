#include "stm32f1xx.h"
#include <stdint.h>

#define TIMEOUT 10000

void I2C1_Init(void);

void I2C_Start (void);

void I2C_Write (uint8_t data);

void I2C_Address (uint8_t Address);

void I2C_Stop (void);

void I2C_WriteMulti (uint8_t *data, uint8_t ssize);

void I2C_Read (uint8_t *bbuffer, uint8_t ssize);
