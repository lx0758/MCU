#ifndef _ENCODER_H__
#define _ENCODER_H__

#ifdef STM32F103xB
#include "stm32f1xx.h"
#else
#include "stm32f4xx.h"
#endif

typedef struct {
    TIM_HandleTypeDef *htim;
    uint8_t frequency;
    int32_t count;
    void (*cb)(int32_t change);
} Encoder_InitTypeDef;

uint8_t ENCODER_Init(Encoder_InitTypeDef *encoder);
uint8_t ENCODER_DeInit(Encoder_InitTypeDef *encoder);
void ENCODER_Scan();

#endif //#ifndef _ENCODER_H__