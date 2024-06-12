#include "i2c.h"

void I2C1_Init(void) {
    RCC->APB2ENR |= 1 << 3; // Enable GIOB
    RCC->APB2ENR |= 1; // Enable AFIO
    RCC->APB1ENR |= 1 << 21; // Enable I2C1

    GPIOB->CRL |= 0xDD<< 24; // set pin 6 and 7 of GPIOB to output mode, alternate function out put open -drain
    
    I2C1->CR2 = 0x24;  // set PCLK frequency to 36Mhz,  is limited by the maximum APB1 frequency  36MHz
    I2C1->CCR = 0xB4;  //  generate 100kHz SCL 
    I2C1->TRISE = 0x25;  // Trise = the maximum SCL rise time given in the I 2 C bus specification, incremented by 1

    I2C1->CR1 |= (1<<0);  // Enable I2C
}

void I2C_Start (void)
{
    uint32_t timeout = TIMEOUT;
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--); //EV5
}

void I2C_Write (uint8_t data)
{
  uint32_t timeout = TIMEOUT;
  while (!(I2C1->SR1 & (1<<I2C_SR1_TXE))&& timeout--);  // wait for TXE bit to set
  I2C1->DR = data;  // wait for BTF bit to set
}

void I2C_Address (uint8_t Address)
{
  uint32_t timeout = TIMEOUT;
  I2C1->DR = Address;
  while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout--); //EV6
  (void)I2C1->SR2; 
}

void I2C_Stop (void)
{
  I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_WriteMulti (uint8_t *data, uint8_t ssize)
{
  for(int i = 0; i<ssize; i ++){
    I2C_Write(data[i]);
  }
}

void I2C_Read(uint8_t* bbuffer, uint8_t ssize) {
    uint32_t timeout = TIMEOUT;

    if (ssize == 1) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        I2C1->CR1 |= I2C_CR1_STOP;

        timeout = TIMEOUT;
        while (!(I2C1->SR1 & I2C_SR1_RXNE) && timeout--);
        if (timeout == 0) return;

        bbuffer[0] = I2C1->DR;
    } else {
        for (uint8_t i = 0; i < ssize - 1; i++) {
            I2C1->CR1 |= I2C_CR1_ACK; // Send ACK
            timeout = TIMEOUT;
            while (!(I2C1->SR1 & I2C_SR1_RXNE) && timeout--);
            if (timeout == 0) return;
            
            bbuffer[i] = I2C1->DR;
        }

        I2C1->CR1 &= ~I2C_CR1_ACK;
        
        timeout = TIMEOUT;
        while (!(I2C1->SR1 & I2C_SR1_RXNE) && timeout--);
        if (timeout == 0) return;

        bbuffer[ssize - 1] = I2C1->DR;
        
    }
}
