#ifndef _BUTTON_H__
#define _BUTTON_H__

#ifdef STM32F103xB
#include "stm32f1xx.h"
#else
#include "stm32f4xx.h"
#endif

typedef enum {
    BUTTON_EVENT_DOWN   = 0,
    BUTTON_EVENT_UP     = 1,
} Button_EventTypeDef;

typedef struct {
    GPIO_TypeDef *type;
    uint16_t pin;
    GPIO_PinState target;
    uint8_t state;
    uint32_t press_time;
    void (*event)(Button_EventTypeDef event);
    void (*press)(void);
    void (*long_press)(void);
} Button_InitTypeDef;

uint8_t BUTTON_Init(Button_InitTypeDef *btn);

uint8_t BUTTON_DeInit(Button_InitTypeDef *btn);

void BUTTON_Scan();

#endif //#ifndef _BUTTON_H__