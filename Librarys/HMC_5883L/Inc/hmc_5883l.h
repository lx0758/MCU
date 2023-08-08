#ifndef HMC_5883L_H
#define HMC_5883L_H

#define USE_MPU_6050
#define MPU6050_ADDRESS 0x68

#ifdef STM32F103xB
#include "stm32f1xx.h"
#else
#include "stm32f4xx.h"
#endif

uint8_t HMC_Init(I2C_HandleTypeDef *hi2c);

uint8_t HMC_GetAngle(float *ax, float *ay, float *az);

#endif //HMC_5883L_H
