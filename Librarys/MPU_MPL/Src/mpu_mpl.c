#include "mpu_mpl.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"

#if defined MPU9150
    #ifndef MPU6050
        #define MPU6050
    #endif
#elif defined MPU9250
    #ifndef MPU6500
        #define MPU6500
    #endif
#endif

#ifndef SELF_TEST_DEBUG
    #define SELF_TEST_DEBUG false
#endif

#define FSR_GYRO      2000
#define FSR_ACCEL     2
#define DEFAULT_RATE  50
#define q30           1073741824.0f
#define q16           65536.0f

unsigned char *mpl_key = "eMPL 5.1";

static signed char orientation_matrix[9] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1,
};

uint8_t inited = 0;
data_size_func_t data_size_func;
data_load_func_t data_load_func;
data_save_func_t data_save_func;

I2C_HandleTypeDef *gMplHi2c;

void MPU_MPL_LoadConfig();
void MPU_MPL_SaveConfig();

uint8_t MPU_MPL_Init(
    I2C_HandleTypeDef *hi2c,
    data_size_func_t size_func,
    data_load_func_t load_func,
    data_save_func_t save_func
) {
  gMplHi2c = hi2c;
  data_size_func = size_func;
  data_load_func = load_func;
  data_save_func = save_func;

  unsigned char accel_fsr;
  unsigned short gyro_fsr, compass_fsr;
  unsigned short sample_rate, compass_sample_rate;

  int result = 0;
  result |= mpu_init(NULL);
  result |= mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  result |= mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  result |= mpu_set_gyro_fsr(FSR_GYRO);
  result |= mpu_set_accel_fsr(FSR_ACCEL);
  result |= mpu_get_gyro_fsr(&gyro_fsr);
  result |= mpu_get_accel_fsr(&accel_fsr);
  result |= mpu_get_compass_fsr(&compass_fsr);
  result |= mpu_set_sample_rate(DEFAULT_RATE);
  result |= mpu_set_compass_sample_rate(DEFAULT_RATE);
  result |= mpu_get_sample_rate(&sample_rate);
  result |= mpu_get_compass_sample_rate(&compass_sample_rate);
  if (result) return 1;

  result |= dmp_load_motion_driver_firmware();
  result |= dmp_enable_feature(
          DMP_FEATURE_6X_LP_QUAT |
          DMP_FEATURE_SEND_RAW_ACCEL |
          DMP_FEATURE_GYRO_CAL |
          DMP_FEATURE_SEND_CAL_GYRO
  );
  result |= dmp_set_fifo_rate(DEFAULT_RATE);
  result |= dmp_set_orientation(inv_orientation_matrix_to_scalar(orientation_matrix));
  result |= mpu_set_dmp_state(true);
  if (result) return 2;

  result |= inv_init_mpl();
  result |= inv_enable_quaternion();
  result |= inv_enable_9x_sensor_fusion();
  result |= inv_enable_fast_nomot();
  result |= inv_enable_gyro_tc();
  result |= inv_enable_vector_compass_cal();
  result |= inv_enable_in_use_auto_calibration();
  result |= inv_enable_magnetic_disturbance();
  result |= inv_enable_dip_tracking();
  result |= inv_enable_heading_from_gyro();
  result |= inv_enable_no_gyro_fusion();
  result |= inv_enable_results_holder();
  result |= inv_enable_hal_outputs();
  inv_set_gyro_sample_rate(1000000L / sample_rate);
  inv_set_accel_sample_rate(1000000L / sample_rate);
  inv_set_compass_sample_rate(1000000L / compass_sample_rate);
  inv_set_gyro_orientation_and_scale(inv_orientation_matrix_to_scalar(orientation_matrix),(long) gyro_fsr << 15);
  inv_set_accel_orientation_and_scale(inv_orientation_matrix_to_scalar(orientation_matrix),(long) accel_fsr << 15);
  inv_set_compass_orientation_and_scale(inv_orientation_matrix_to_scalar(orientation_matrix),(long) compass_fsr << 15);
  MPU_MPL_LoadConfig();
  result |= inv_start_mpl();
  if (result) return 3;

  inv_get_sensor_type_gravity(NULL, 0, 0);
  inited = 1;
  return 0;
}

uint8_t MPU_MPL_RunSelfTest() {
    long gyro_bias[3] = {0}, accel_bias[3] = {0};
#ifdef MPU6500
    int result = mpu_run_6500_self_test(gyro_bias, accel_bias, SELF_TEST_DEBUG);
    if ((result & 0b111) != 0b111) {
        return -1;
    }
#else
    int result = mpu_run_self_test(gyro_bias, accel_bias);
  if ((result & 0b11) != 0b11) {
    return -1;
  }
#endif

    result = 0;

    float gyro_sens;
    result |= mpu_get_gyro_sens(&gyro_sens);
    gyro_bias[0] *= (long)(((float) gyro_bias[0]) * gyro_sens);
    gyro_bias[1] *= (long)(((float) gyro_bias[1]) * gyro_sens);
    gyro_bias[2] *= (long)(((float) gyro_bias[2]) * gyro_sens);
    dmp_set_gyro_bias(gyro_bias);
    inv_set_gyro_bias(gyro_bias, 3);

    unsigned short accel_sens;
    result |= mpu_get_accel_sens(&accel_sens);
    accel_bias[0] *= accel_sens;
    accel_bias[1] *= accel_sens;
    accel_bias[2] *= accel_sens;
    dmp_set_accel_bias(accel_bias);
    inv_set_accel_bias(accel_bias, 3);

    MPU_MPL_SaveConfig();

    return result;
}

uint8_t MPU_MPL_HandlerData() {
  uint8_t result = 0;

  if (!inited) return -1;

  short gyro[3] = {0}, accel[3] = {0}, sensors = 0;
  long quat[4] = {0};
  unsigned long timestamp = 0;
  unsigned char more = 0;
  if (!dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)) {
    if (sensors & INV_XYZ_ACCEL) {
      long lAccel[] = {accel[0], accel[1], accel[2]};
      result |= inv_build_accel(lAccel, 0, timestamp);
    }
    if (sensors & INV_XYZ_GYRO) {
      result |= inv_build_gyro(gyro, timestamp);
      long temperature = {0};
      if (!mpu_get_temperature(&temperature, &timestamp)) {
        result |= inv_build_temp(temperature, timestamp);
      }
    }
    result |= inv_build_quat(quat, 0, timestamp);
  }

#ifdef MPU6500
  short compass[3] = {0};
  if (!mpu_get_compass_reg(compass, &timestamp)) {
    long lCompass[] = {compass[0], compass[1], compass[2]};
    result |= inv_build_compass(lCompass,0, timestamp);
  }
#endif

  result |= inv_execute_on_data();

  return result;
}

uint8_t MPU_MPL_GetOrientation(float *pitch, float *roll, float *azimuth, int8_t *accuracy) {
  if (!inited) {
    *pitch = -1;
    *roll = -1;
    *azimuth = -1;
    *accuracy = 0;
    return -1;
  }
  float data[3];
  inv_time_t time;
  uint8_t result = inv_get_sensor_type_orientation(data, accuracy, &time);
  *pitch = data[1];
  *roll = data[2];
  *azimuth = data[0];
  return result;
}

uint8_t stm32_i2c_write_byte(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
  return HAL_I2C_Mem_Write(gMplHi2c, slave_addr << 1, reg_addr, 1, data, length, 0xFF);
}

uint8_t stm32_i2c_read_byte(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
  return HAL_I2C_Mem_Read(gMplHi2c, slave_addr << 1, reg_addr, 1, data, length, 0xFF);
}

uint8_t stm32_delay_ms(unsigned long num_ms) {
  HAL_Delay(num_ms);
  return 0;
}

uint8_t stm32_get_ms(unsigned long *count) {
  *count = HAL_GetTick();
  return 0;
}

void MPU_MPL_LoadConfig() {
  size_t size = data_size_func();
  uint8_t data[size];
  if (!data_load_func(data, size)) {
      inv_load_mpl_states(data, size);
  }
}

void MPU_MPL_SaveConfig() {
  uint8_t result;
  size_t size = 0;
  result = inv_get_mpl_state_size(&size);
  if (result != INV_SUCCESS) return;

  uint8_t data[size];
  result = inv_save_mpl_states(data, size);
  if (result != INV_SUCCESS) return;

  result = data_save_func(data, size);
  if (result) return;
}