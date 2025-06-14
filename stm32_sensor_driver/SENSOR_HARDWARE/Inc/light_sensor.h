#ifndef _LTR390_OPTIMIZED_H_
#define _LTR390_OPTIMIZED_H_

/*
	感光传感器，包含了uvs和als数据，读取紫外线和光强度数据, 使用i2c协议通讯
	传感器型号:LTR390
	协议类型：i2c

	Photosensitive sensor providing both UVS and ALS data, measuring ultraviolet and light 		intensity, using I2C protocol for communication. 
	Sensor model: LTR390
	Protocol type: i2c
	
	SDA:pb9
	SCL:pb8

*/


#include <stdint.h>
void Test_UVS_With_Debug(void);
uint8_t LTR390_Init(void);
uint8_t LTR390_ReadAls(float* als_data);
uint8_t LTR390_ReadUvs(float* uvs_data);

#endif
