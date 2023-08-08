#ifndef FLASH_H
#define FLASH_H

#ifdef STM32F103xB
#include "stm32f1xx.h"
#else
#include "stm32f4xx.h"
#endif

#define W25Q32

uint8_t FLASH_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

uint32_t FLASH_GetDeviceId();
uint32_t FLASH_GetCapacitySize();
uint8_t FLASH_Erase_4kb(uint32_t addr);
uint8_t FLASH_Erase_32kb(uint32_t addr);
uint8_t FLASH_Erase_64kb(uint32_t addr);
uint8_t FLASH_Erase_All();
uint8_t FLASH_Erase_Sleep();
uint8_t FLASH_Erase_Wakeup();

uint8_t FLASH_ReadByte(uint32_t addr);
uint8_t FLASH_Read(uint32_t addr, uint8_t *pBuff, uint32_t size);

uint8_t FLASH_WriteByte(uint32_t addr, uint8_t data);
uint8_t FLASH_Write(uint32_t addr, uint8_t *data, uint32_t size);

#endif //FLASH_H
