/**
 * X25Q 系列芯片是一种串行 NOR Flash 存储器
 * 同一时间最多写一页
 * 擦除操作只能 按扇区擦除 按块擦除 整个擦除
 *
 * 编程即写数据，由于 Flash 的特性只能从 1 编程 0，
 * 所以写数据之前 Flash 里面的数据不是 0xFF 就必须先擦除，然后才能写数据。
 *
 * 1页(Page)     = 256 Bytes
 * 1扇(Sector)   = 16页       =  4KBytes
 * 1块(Block)    = 16扇       = 64KBytes
 *
 * 25Q80   =  8MB  = 1Mb   = 16块   = 256扇  =  4096页
 * 25Q16   = 16MB  = 2Mb   = 32块   = 512扇  =  8192页
 * 25Q32   = 32MB  = 4Mb   = 64块   =1024扇  = 16384页
 * 25Q64   = 64MB  = 8Mb   =128块   =2048扇  = 32768页
 * 25Q128  =128MB  =16Mb   =256块   =4096扇  = 65536页
 * 25Q256  =256MB  =32Mb   =512块   =8192扇  =131072页
 *
 * https://www.findic.com/article/96537.html
 */
#include "flash.h"

#ifdef W25Q80
  #define FLASH_CAPACITY_SIZE	(1 * 1024 * 1024)
  #define FLASH_ADDR_BYTE_NUM	3
#endif
#ifdef W25Q16
  #define FLASH_CAPACITY_SIZE	(2 * 1024 * 1024)
  #define FLASH_ADDR_BYTE_NUM	3
#endif
#ifdef W25Q32
  #define FLASH_CAPACITY_SIZE	(4 * 1024 * 1024)
  #define FLASH_ADDR_BYTE_NUM	3
#endif
#ifdef W25Q64
  #define FLASH_CAPACITY_SIZE	(8 * 1024 * 1024)
  #define FLASH_ADDR_BYTE_NUM	3
#endif
#ifdef W25Q128
  #define FLASH_CAPACITY_SIZE	(16 * 1024 * 1024)
  #define FLASH_ADDR_BYTE_NUM	3
#endif
#ifdef W25Q256
  #define FLASH_CAPACITY_SIZE	(32 * 1024 * 1024)
  #define FLASH_ADDR_BYTE_NUM	4
#endif

#define FLASH_PAGE_SIZE           256
#define FLASH_SECTOR_SIZE         4096

#define W25X_READ_STATUS_REG_1    0x05
#define W25X_READ_STATUS_REG_2    0x35
#define W25X_READ_STATUS_REG_3    0x15
#define W25X_WRITE_STATUS_REG_1   0x01
#define W25X_WRITE_STATUS_REG_2   0x31
#define W25X_WRITE_STATUS_REG_3   0x11

#define W25X_SLEEP                0xB9
#define W25X_WAKEUP               0xAB
#define W25X_READ_JEDEC_ID        0x9F

#define W25X_ENABLE_4_BYTE_ADDR   0xB7
#define W25X_DISABLE_4_BYTE_ADDR  0xE9

#define W25X_WRITE_ENABLE         0x06
#define W25X_WRITE_DISABLE        0x04

#define W25X_ERASE_SECTOR         0x20
#define W25X_ERASE_BLOCK_32K      0x52
#define W25X_ERASE_BLOCK_64K      0xD8
#define W25X_ERASE_CHIP           0xC7

#define W25X_READ_DATA            0x03
#define W25X_PAGE_PROGRAM         0x02

SPI_HandleTypeDef *gNorFlashHspi;
GPIO_TypeDef *gNorFlashGPIOx;
uint16_t gNorFlashGPIO_Pin;

void FLASH_CS_Low();
void FLASH_CS_High();

uint8_t FLASH_SendCmd(uint8_t cmd);
uint8_t FLASH_SendAddr(uint32_t addr);
uint8_t FLASH_SendData(uint8_t *data, uint16_t size);
uint8_t FLASH_ReceiveData(uint8_t *data, uint16_t size);

/************************ AUTO_CS_BEGIN ************************/
uint8_t FLASH_WriteEnable();
uint8_t FLASH_WriteDisable();
uint8_t FLASH_ReadRegister(uint8_t cmd);
uint8_t FLASH_WriteRegister(uint8_t cmd, uint8_t value);
uint8_t FLASH_WaitIdle();
uint8_t FLASH_Erase(uint8_t cmd, uint32_t addr);
/************************  AUTO_CS_END  ************************/

uint8_t FLASH_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
  gNorFlashHspi = hspi;
  gNorFlashGPIOx = GPIOx;
  gNorFlashGPIO_Pin = GPIO_Pin;

  FLASH_CS_High();
  // 上电后第一次读写操作失效
  // 可以通过调用一次读取来规避
  FLASH_GetDeviceId();

  if (FLASH_ADDR_BYTE_NUM == 4) {
    if ((FLASH_ReadRegister(W25X_READ_STATUS_REG_3) & 0x01) == 0) {
      FLASH_CS_Low();
      FLASH_SendCmd(W25X_ENABLE_4_BYTE_ADDR);
      FLASH_CS_High();
    }
  }
  return SUCCESS;
}

uint32_t FLASH_GetDeviceId() {
  uint32_t result = 0;
  FLASH_CS_Low();
  FLASH_SendCmd(W25X_READ_JEDEC_ID);
  FLASH_SendAddr(0x00);
  uint8_t pBuff[3] = {0};
  FLASH_ReceiveData(pBuff, 3);
  result |= pBuff[0] << 16;
  result |= pBuff[1] << 8;
  result |= pBuff[2];
  FLASH_CS_High();
  return result;
}

uint32_t FLASH_GetCapacitySize() {
  return FLASH_CAPACITY_SIZE;
}

uint8_t FLASH_Erase_4kb(uint32_t addr) {
  addr = addr / 0x1000 * 0x1000;
  return FLASH_Erase(W25X_ERASE_SECTOR, addr);
}

uint8_t FLASH_Erase_32kb(uint32_t addr) {
  addr = addr / 0x8000 * 0x8000;
  return FLASH_Erase(W25X_ERASE_BLOCK_32K, addr);
}

uint8_t FLASH_Erase_64kb(uint32_t addr) {
  addr = addr / 0xFFFF * 0xFFFF;
  return FLASH_Erase(W25X_ERASE_BLOCK_64K, addr);
}

uint8_t FLASH_Erase_All() {
  return FLASH_Erase(W25X_ERASE_CHIP, 0);
}

uint8_t FLASH_Erase_Sleep() {
  uint8_t result = SUCCESS;
  FLASH_CS_Low();
  result |= FLASH_SendCmd(W25X_SLEEP);
  FLASH_CS_High();
  return result;
}

uint8_t FLASH_Erase_Wakeup() {
  uint8_t result = SUCCESS;
  FLASH_CS_Low();
  result |= FLASH_SendCmd(W25X_WAKEUP);
  FLASH_CS_High();
  return result;
}

uint8_t FLASH_ReadByte(uint32_t addr) {
  uint8_t result = 0;
  FLASH_Read(addr, &result, 1);
  return result;
}

uint8_t FLASH_Read(uint32_t addr, uint8_t *pBuff, uint32_t size) {
  uint8_t result = SUCCESS;
  FLASH_CS_Low();
  result |= FLASH_SendCmd(W25X_READ_DATA);
  result |= FLASH_SendAddr(addr);
  result |= FLASH_ReceiveData(pBuff, size);
  FLASH_CS_High();
  return result;
}

uint8_t FLASH_WriteByte(uint32_t addr, uint8_t data) {
  return FLASH_Write(addr, &data, 1);
}

uint8_t FLASH_Write(uint32_t addr, uint8_t *data, uint32_t size) {
  uint8_t result = SUCCESS;

  uint8_t *dataCopy = data;
  uint32_t beginAddr = addr, endAddr = addr + size;
  uint32_t needWriteSize, canWriteSize, writeSize;

  if (endAddr > FLASH_CAPACITY_SIZE) {
    endAddr = FLASH_CAPACITY_SIZE;
  }

  while (beginAddr < endAddr) {
    needWriteSize = endAddr - beginAddr;
    canWriteSize = FLASH_PAGE_SIZE - beginAddr % FLASH_PAGE_SIZE;
    writeSize = needWriteSize <= canWriteSize ? needWriteSize : canWriteSize;

    result |= FLASH_WriteEnable();
    FLASH_CS_Low();
    result |= FLASH_SendCmd(W25X_PAGE_PROGRAM);
    result |= FLASH_SendAddr(beginAddr);
    result |= FLASH_SendData(dataCopy, writeSize);
    FLASH_CS_High();
    result |= FLASH_WaitIdle();
    if (result) break;

    dataCopy += writeSize;
    beginAddr += writeSize;
  }

  return result;
}

void FLASH_CS_Low() {
  HAL_GPIO_WritePin(gNorFlashGPIOx, gNorFlashGPIO_Pin, GPIO_PIN_RESET);
}

void FLASH_CS_High() {
  HAL_GPIO_WritePin(gNorFlashGPIOx, gNorFlashGPIO_Pin, GPIO_PIN_SET);
}

uint8_t FLASH_SendCmd(uint8_t cmd) {
  return FLASH_SendData(&cmd, 1);
}

uint8_t FLASH_SendAddr(uint32_t addr) {
  uint8_t data[FLASH_ADDR_BYTE_NUM] = {0};
  for (int i = 0; i < FLASH_ADDR_BYTE_NUM; ++i) {
    data[i] = addr >> ((FLASH_ADDR_BYTE_NUM - i - 1) * 8);
  }
  return FLASH_SendData(data, FLASH_ADDR_BYTE_NUM);
}

uint8_t FLASH_SendData(uint8_t *data, uint16_t size) {
  return HAL_SPI_Transmit(gNorFlashHspi, data, size, 0xFF);
}

uint8_t FLASH_ReceiveData(uint8_t *data, uint16_t size) {
  return HAL_SPI_Receive(gNorFlashHspi, data, size, 0xFF);
}

uint8_t FLASH_WriteEnable() {
  uint8_t result = SUCCESS;
  FLASH_CS_Low();
  result |= FLASH_SendCmd(W25X_WRITE_ENABLE);
  FLASH_CS_High();
  return result;
}

uint8_t FLASH_WriteDisable() {
  uint8_t result = SUCCESS;
  FLASH_CS_Low();
  result |= FLASH_SendCmd(W25X_WRITE_DISABLE);
  FLASH_CS_High();
  return result;
}

uint8_t FLASH_ReadRegister(uint8_t cmd) {
  uint8_t result = 0;
  FLASH_CS_Low();
  FLASH_SendCmd(cmd);
  FLASH_ReceiveData(&result, 1);
  FLASH_CS_High();
  return result;
}

uint8_t FLASH_WriteRegister(uint8_t cmd, uint8_t value) {
  uint8_t result = SUCCESS;
  result |= FLASH_WriteEnable();
  FLASH_CS_Low();
  result |= FLASH_SendCmd(cmd);
  result |= FLASH_SendData(&value, 1);
  FLASH_CS_High();
  result |= FLASH_WaitIdle();
  return result;
}

uint8_t FLASH_WaitIdle() {
  while ((FLASH_ReadRegister(W25X_READ_STATUS_REG_1) & 0x01) == 0x01) {
    HAL_Delay(5);
  }
  return SUCCESS;
}

uint8_t FLASH_Erase(uint8_t cmd, uint32_t addr) {
  uint8_t result = SUCCESS;
  result |= FLASH_WriteEnable();
  FLASH_CS_Low();
  result |= FLASH_SendCmd(cmd);
  if (cmd != W25X_ERASE_CHIP) {
    result |= FLASH_SendAddr(addr);
  }
  FLASH_CS_High();
  result |= FLASH_WaitIdle();
  return result;
}