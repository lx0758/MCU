#ifndef EEPROM_H
#define EEPROM_H

#ifdef STM32F103xB
#include "stm32f1xx.h"
#else
#include "stm32f4xx.h"
#endif

/**
 * X24C01
 * X24C02
 * X24C04
 * X24C08
 * X24C16
 * X24C32
 * X24C64
 * X24C128
 * X24C256
 * X24C512
 */
#define X24C02
#define EEPROM_ADDR  0xA0

uint8_t EEPROM_Init(I2C_HandleTypeDef *hi2c);

uint16_t EEPROM_GetCapacitySize();

uint8_t EEPROM_ReadByte(uint16_t addr);
uint8_t EEPROM_Read(uint16_t addr, uint8_t *pBuff, uint16_t size);

uint8_t EEPROM_WriteByte(uint16_t addr, uint8_t data);
uint8_t EEPROM_Write(uint16_t addr, uint8_t *data, uint16_t size);

#endif //EEPROM_H
