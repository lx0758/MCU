#ifndef _SYSCALLSEXT_H__
#define _SYSCALLSEXT_H__

#ifdef STM32F103xB
#include "stm32f1xx.h"
#else
#include "stm32f4xx.h"
#endif

void SYSCALLSEXT_Init(UART_HandleTypeDef *huart);

#endif //#ifndef _SYSCALLSEXT_H__