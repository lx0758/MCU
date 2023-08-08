#ifndef NMEA0183_H
#define NMEA0183_H

#ifdef STM32F103xB
#include "stm32f1xx.h"
#else
#include "stm32f4xx.h"
#endif

typedef struct {
    long utc_time;
    double latitude;
    char latitude_direction;
    double longitude;
    char longitude_direction;
    float altitude;
    float ellipsoid;
    char state;
    uint8_t satellite_available;
    uint8_t bds_satellite_total;
    uint8_t gal_satellite_total;
    uint8_t glo_satellite_total;
    uint8_t gps_satellite_total;
} NmeaInfo;

void NMEA_Init(UART_HandleTypeDef *huart);

void NMEA_GetInfo(NmeaInfo *nmeaInfo);

#endif //NMEA0183_H
