#ifndef MPU6050_H
#define MPU6050_H

#ifdef STM32F103xB
#include "stm32f1xx.h"
#else
#include "stm32f4xx.h"
#endif

void MPU_Init(I2C_HandleTypeDef *hi2c);

uint8_t MPU_GetGyroscope(float *gx, float *gy, float *gz);
uint8_t MPU_GetAccelerometer(float *ax, float *ay, float *az);
uint8_t MPU_GetTemperature(float *temperature);

#endif //MPU6050_H
