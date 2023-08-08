/**
 * X24C 系列芯片是串行电气可擦除和可编程只读存储器(EEPROM)
 *
 * 24C01    =1kB    =128b   =16页  x 8b
 * 24C02    =2kB    =256b   =32页  x 8b
 * 24C04    =4kB    =512b   =32页  x 16b
 * 24C08    =8kB    =1kb    =64页  x 16b
 * 24C16    =16kB   =2kb    =128页 x 16b
 * 24C32    =32kB   =4kb    =128页 x 32b
 * 24C64    =64kB   =8kb    =256页 x 32b
 * 24C128   =128kB  =16kb   =256页 x 64b
 * 24C256   =256kB  =32kb   =512页 x 64b
 * 24C512   =512kB  =64kb   =512页 x 128b
 *
 * https://blog.csdn.net/wanglong3713/article/details/124579713
 * https://blog.csdn.net/wanglong3713/article/details/124618724
 * https://blog.csdn.net/wanglong3713/article/details/124909042
 */
#include "eeprom.h"

#define EEPROM_WRITE_WAIT_TIME 10                                     // 单次写等待

#ifdef X24C01
  #define EEPROM_PAGE_NUM  			16						                        // 页数
	#define EEPROM_PAGE_SIZE			8						                          // 页面大小(字节)
	#define EEPROM_CAPACITY_SIZE  (EEPROM_PAGE_NUM * EEPROM_PAGE_SIZE)	// 总容量(字节)
	#define EEPROM_ADDR_BYTE_NUM  1						                          // 地址字节个数
#endif
#ifdef X24C02
  #define EEPROM_PAGE_NUM  			32						                        // 页数
	#define EEPROM_PAGE_SIZE			8						                          // 页面大小(字节)
	#define EEPROM_CAPACITY_SIZE	(EEPROM_PAGE_NUM * EEPROM_PAGE_SIZE)  // 总容量(字节)
	#define EEPROM_ADDR_BYTE_NUM	1						                          // 地址字节个数
#endif
#ifdef X24C04
  #define EEPROM_PAGE_NUM  			32						                        // 页数
	#define EEPROM_PAGE_SIZE			16						                        // 页面大小(字节)
	#define EEPROM_CAPACITY_SIZE  (EEPROM_PAGE_NUM * EEPROM_PAGE_SIZE)  // 总容量(字节)
	#define EEPROM_ADDR_BYTE_NUM  1						                          // 地址字节个数
#endif
#ifdef X24C08
  #define EEPROM_PAGE_NUM  			64						                        // 页数
	#define EEPROM_PAGE_SIZE			16						                        // 页面大小(字节)
	#define EEPROM_CAPACITY_SIZE  (EEPROM_PAGE_NUM * EEPROM_PAGE_SIZE)  // 总容量(字节)
	#define EEPROM_ADDR_BYTE_NUM  1						                          // 地址字节个数
#endif
#ifdef X24C16
  #define EEPROM_PAGE_NUM  		  128						                        // 页数
	#define EEPROM_PAGE_SIZE		  16						                        // 页面大小(字节)
	#define EEPROM_CAPACITY_SIZE  (EEPROM_PAGE_NUM * EEPROM_PAGE_SIZE)  // 总容量(字节)
	#define EEPROM_ADDR_BYTE_NUM  1					                            // 地址字节个数
#endif
#ifdef X24C32
  #define EEPROM_PAGE_NUM       128						                        // 页数
  #define EEPROM_PAGE_SIZE      32						                        // 页面大小(字节)
  #define EEPROM_CAPACITY_SIZE  (EEPROM_PAGE_NUM * EEPROM_PAGE_SIZE)  // 总容量(字节)
  #define EEPROM_ADDR_BYTE_NUM  2						                          // 地址字节个数
#endif
#ifdef X24C64
  #define EEPROM_PAGE_NUM  			256						                        // 页数
	#define EEPROM_PAGE_SIZE			32						                        // 页面大小(字节)
	#define EEPROM_CAPACITY_SIZE  (EEPROM_PAGE_NUM * EEPROM_PAGE_SIZE)  // 总容量(字节)
	#define EEPROM_ADDR_BYTE_NUM  2						                          // 地址字节个数
#endif
#ifdef X24C128
  #define EEPROM_PAGE_NUM  			256						                        // 页数
	#define EEPROM_PAGE_SIZE			64						                        // 页面大小(字节)
	#define EEPROM_CAPACITY_SIZE  (EEPROM_PAGE_NUM * EEPROM_PAGE_SIZE)  // 总容量(字节)
	#define EEPROM_ADDR_BYTE_NUM  2						                          // 地址字节个数
#endif
#ifdef X24C256
  #define EEPROM_PAGE_NUM  			512						                        // 页数
	#define EEPROM_PAGE_SIZE			64						                        // 页面大小(字节)
	#define EEPROM_CAPACITY_SIZE  (EEPROM_PAGE_NUM * EEPROM_PAGE_SIZE)  // 总容量(字节)
	#define EEPROM_ADDR_BYTE_NUM  2						                          // 地址字节个数
#endif
#ifdef X24C512
  #define EEPROM_PAGE_NUM  			512						                        // 页数
	#define EEPROM_PAGE_SIZE			128						                        // 页面大小(字节)
	#define EEPROM_CAPACITY_SIZE  (EEPROM_PAGE_NUM * EEPROM_PAGE_SIZE)  // 总容量(字节)
	#define EEPROM_ADDR_BYTE_NUM  2						                          // 地址字节个数
#endif

I2C_HandleTypeDef *gEepromHi2c;

uint8_t EEPROM_Init(I2C_HandleTypeDef *hi2c) {
  gEepromHi2c = hi2c;
  return SUCCESS;
}

uint16_t EEPROM_GetCapacitySize() {
  return EEPROM_CAPACITY_SIZE;
}

uint8_t EEPROM_ReadByte(uint16_t addr) {
  uint8_t result = 0;
  HAL_I2C_Mem_Read(gEepromHi2c, EEPROM_ADDR, addr, EEPROM_ADDR_BYTE_NUM, &result, 1, 0xFF);
  return result;
}

uint8_t EEPROM_Read(uint16_t addr, uint8_t *pBuff, uint16_t size) {
  if (addr >= EEPROM_CAPACITY_SIZE) return -1;
  return HAL_I2C_Mem_Read(gEepromHi2c, EEPROM_ADDR, addr, EEPROM_ADDR_BYTE_NUM, pBuff, size, 0xFF);
}

uint8_t EEPROM_WriteByte(uint16_t addr, uint8_t data) {
  if (addr >= EEPROM_CAPACITY_SIZE) return ERROR;
  uint8_t result = HAL_I2C_Mem_Write(gEepromHi2c, EEPROM_ADDR, addr, EEPROM_ADDR_BYTE_NUM, &data, 1, 0xFF);
  HAL_Delay(EEPROM_WRITE_WAIT_TIME);
  return result;
}

uint8_t EEPROM_Write(uint16_t addr, uint8_t *data, uint16_t size) {
  uint8_t result = SUCCESS;

  uint8_t *dataCopy = data;
  uint16_t beginAddr = addr, endAddr = addr + size;
  uint16_t needWriteSize, canWriteSize, writeSize;

  if (endAddr > EEPROM_CAPACITY_SIZE) {
    endAddr = EEPROM_CAPACITY_SIZE;
  }

  while (beginAddr < endAddr) {
    needWriteSize = endAddr - beginAddr;
    canWriteSize = EEPROM_PAGE_SIZE - beginAddr % EEPROM_PAGE_SIZE;
    writeSize = needWriteSize <= canWriteSize ? needWriteSize : canWriteSize;

    result |= HAL_I2C_Mem_Write(gEepromHi2c, EEPROM_ADDR, beginAddr, EEPROM_ADDR_BYTE_NUM, dataCopy, writeSize, 0xFF);
    HAL_Delay(EEPROM_WRITE_WAIT_TIME);
    if (result) break;

    dataCopy += writeSize;
    beginAddr += writeSize;
  }

  return result;
}