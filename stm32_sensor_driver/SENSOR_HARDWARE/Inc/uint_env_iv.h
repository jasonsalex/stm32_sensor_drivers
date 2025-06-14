#ifndef UINT_ENV_IV_H
#define UINT_ENV_IV_H

/*
	
	温湿度传感器，包含了sht40湿度，温度以及bmp280气压传感器，使用i2c协议通讯
	传感器型号:enviv sensor(sht40, bmp280)
	协议类型:i2c

	Temperature and humidity sensor, including SHT40 humidity, temperature, and BMP280 		barometric pressure sensors, using I2C protocol for communication. 
	Sensor model: enviv sensor(sht40, bmp280)
	Protocol type: i2c

	SDA:pb13
	SCL:pb14
*/

#include <stdint.h>

typedef struct {
    // 温度校准参数
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    
    // 气压校准参数
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} BMP280_Calib;

void EnvIV_SensorInit(void);

uint8_t SHT40_ReadTempHum(float *temp_ptr, float *hum_ptr);

void SHT40_Sleep(void);
void SHT40_StartMeasurement(void);
void SHT40_Write(uint8_t command);

void BMP280_CalibInit(void);
float BMP280_ReadPressureWithCompensation(void) ;
uint8_t BMP280_Detect(void);

uint8_t CheckCRC8(uint8_t *data, uint8_t len);
#endif
