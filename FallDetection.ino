#include "i2c.h"
#include <math.h>

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
typedef enum {
    WORKING,
    STOP,
    FALL
} State;
State state = WORKING;

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

float Ax, Ay, Az;
float prevAx, prevAy, prevAz;
float magnitude;
bool firstRun = true;
uint8_t check;
uint8_t prevSW1 = 1;
uint8_t prevSW2 = 1;
uint8_t SW1, SW2;
uint8_t led1Clk = 0, led2Clk = 0;
void MPU_Write (uint8_t Address, uint8_t Reg, uint8_t Data)
{
  I2C_Start();
  I2C_Address (Address);
  I2C_Write (Reg);
  I2C_Write (Data);
  I2C_Stop ();
}
void MPU_Read (uint8_t Address, uint8_t Reg, uint8_t *bbuffer, uint8_t ssize)
{
  I2C_Start ();
  I2C_Address (Address);
  I2C_Write (Reg);
  I2C_Start ();  // repeated start
  I2C_Address (Address + 0x01);
  I2C_Read ( bbuffer, ssize);
  I2C_Stop ();
}
void MPU6050_Init (void)
{
  uint8_t check;
  uint8_t Data;
  MPU_Read (MPU6050_ADDR,WHO_AM_I_REG, &check, 1);
  if (check == 104)  // 0x68 if ok
  {
    Data = 0;
    MPU_Write (MPU6050_ADDR, PWR_MGMT_1_REG, Data);

    
    Data = 0x07;  //sample rate 1KHz
    MPU_Write(MPU6050_ADDR, SMPLRT_DIV_REG, Data);

    Data = 0x00;
    MPU_Write(MPU6050_ADDR, ACCEL_CONFIG_REG, Data);
  }
}

void MPU6050_Read_Accel (void)
{
  uint8_t data[6];
  MPU_Read (MPU6050_ADDR, ACCEL_XOUT_H_REG, data, 6);

  Accel_X_RAW = (int16_t)(data[0] << 8 | data [1]);
  Accel_Y_RAW = (int16_t)(data[2] << 8 | data [3]);
  Accel_Z_RAW = (int16_t)(data[4] << 8 | data [5]);
  Ax = Accel_X_RAW/16384.0* 9.8;
  Ay = Accel_Y_RAW/16384.0* 9.8;
  Az = Accel_Z_RAW/16384.0* 9.8;
  if(!firstRun){
    magnitude = sqrt((Ax - prevAx) * (Ax - prevAx) + (Ay - prevAy) * (Ay - prevAy) + (Az - prevAz) * (Az - prevAz));
  }
  else
    firstRun = false;
  prevAx = Ax;
  prevAy = Ay;
  prevAz = Az;
  
}
void setup() {
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;

  //PA0, PA1 as inputs pull-down
  GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 | GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
  GPIOA->CRL |= (GPIO_CRL_CNF0_1 | GPIO_CRL_CNF1_1); // Input mode, pull-down
  GPIOA->ODR &= ~((1 << 0) | (1 << 1));
  
  // PB13,PB12 as output
  GPIOB->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12| GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
  GPIOB->CRH |= GPIO_CRH_MODE12_1 | GPIO_CRH_MODE12_0| GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0; 
  
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  TIM2->PSC = 720 - 1; //timer clock 100 kHz
  TIM2->CR1 |= TIM_CR1_ARPE;
  TIM2->ARR = 10000 - 1;
  TIM2->DIER |= TIM_DIER_UIE;
  TIM2->CR1 |= TIM_CR1_CEN;

  I2C1_Init ();
  MPU6050_Init ();
}
void loop() {
  if ((TIM2->SR & TIM_SR_UIF) != 0) { 
      TIM2->SR &= ~TIM_SR_UIF;    
      led1Clk++; 
      led1Clk = led1Clk % 10;
      led2Clk++; 
      led2Clk = led2Clk % 5;
      SW1 = (GPIOA->IDR & GPIO_IDR_IDR1) > 0;
      SW2 = (GPIOA->IDR & GPIO_IDR_IDR0) > 0;
      if(state == WORKING){
    
        MPU6050_Read_Accel();
        
        if(led1Clk == 0)
          GPIOB-> ODR ^= 1 << 13;
        if(magnitude > 20){
          for(int i = 0; i<10; i++){
            while ((TIM2->SR & TIM_SR_UIF) == 0) {}
            TIM2->SR &= ~TIM_SR_UIF;
          }

          int sum = 0;
          for(int i = 0; i<10; i++){
            while ((TIM2->SR & TIM_SR_UIF) == 0) { }
            TIM2->SR &= ~TIM_SR_UIF;
            MPU6050_Read_Accel();
            sum += magnitude;
          }
          if(sum / 10 < 5){
            state = FALL;
            GPIOB-> ODR &= ~(1 << 13);
          }
        }
        if(SW1 == 1 && prevSW1 ==0){
          state = STOP;
          GPIOB-> ODR &= ~(1 << 13);
          GPIOB-> ODR &= ~(1 << 12);
        }
      } else if (state == STOP){
        if(SW1 == 1 && prevSW1 ==0)
          state = WORKING;
      } else if (state == FALL){
        if(led2Clk == 0 )
          GPIOB-> ODR ^= 1 << 12;
    
        if(SW2 == 1 && prevSW2 == 0){
          state = WORKING;
          GPIOB-> ODR &= ~(1 << 12);
        }
      }
      prevSW1 = SW1;
      prevSW2 = SW2;
  }
}
