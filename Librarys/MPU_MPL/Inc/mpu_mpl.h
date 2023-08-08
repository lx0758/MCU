/**
 * 参考文章
 * https://blog.csdn.net/qq_30938259/article/details/122953550
 * 三方库来源 [当前版本 eMD 6.12]
 * https://invensense.tdk.com/developers/software-downloads/
 *
 */
#ifndef MPU_MPL_H
#define MPU_MPL_H

#ifdef STM32F103xB
#include "stm32f1xx.h"
#else
#include "stm32f4xx.h"
#endif

#define i2c_write   stm32_i2c_write_byte
#define i2c_read    stm32_i2c_read_byte
#define delay_ms    stm32_delay_ms
#define get_ms      stm32_get_ms
#define log_i  printf
#define log_e  printf
#define min(a,b) ((a<b)?a:b)

typedef size_t (*data_size_func_t)();
typedef uint8_t (*data_load_func_t)(uint8_t *data, size_t size);
typedef uint8_t (*data_save_func_t)(uint8_t *data, size_t size);

uint8_t MPU_MPL_Init(
    I2C_HandleTypeDef *hi2c,
    data_size_func_t size_func,
    data_load_func_t load_func,
    data_save_func_t save_func
);
uint8_t MPU_MPL_RunSelfTest();
uint8_t MPU_MPL_HandlerData();
uint8_t MPU_MPL_GetOrientation(float *pitch, float *roll, float *azimuth, int8_t *accuracy);

uint8_t stm32_i2c_write_byte(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
uint8_t stm32_i2c_read_byte(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
uint8_t stm32_delay_ms(unsigned long num_ms);
uint8_t stm32_get_ms(unsigned long *count);

#endif //MPU_MPL_H
