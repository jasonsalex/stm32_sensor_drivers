#ifndef _LTR390_OPTIMIZED_H_
#define _LTR390_OPTIMIZED_H_

/*
	�й⴫������������uvs��als���ݣ���ȡ�����ߺ͹�ǿ������, ʹ��i2cЭ��ͨѶ
	�������ͺ�:LTR390
	Э�����ͣ�i2c

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
