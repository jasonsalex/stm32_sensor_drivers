#ifndef __DUST_SENSOR_H
#define __DUST_SENSOR_H

#include <stdint.h>

/*
	���������������������еķ۳����������pm1.0,pm2.5,pm10���ֱ���Ϊ��g/m3..
	�������ͺ�:zh06
	Э�����ͣ�uart

	Air quality sensor for measuring airborne dust concentration, including PM1.0, PM2.5, 		and PM10, with a resolution of ��g/m3
	
	Sensor model: ZH06 
	Protocol type: UART

	SDA:pb11
	SCL:pb10
*/

// ZH06 ���ݽṹ��
typedef struct {
    uint8_t header1;      // ��ʼ�ֽ�1 (0x42)
    uint8_t header2;      // ��ʼ�ֽ�2 (0x4D)
    uint16_t frame_len;   // ֡����
    uint16_t pm1_0;       // PM1.0Ũ�� (��g/m3)
    uint16_t pm2_5;       // PM2.5Ũ�� (��g/m3)
    uint16_t pm10;        // PM10Ũ�� (��g/m3)
    uint16_t checksum;    // У��ֵ (��8λ+��8λ)
} ZH06_Data;

void Dust_SensorInit(void);
void ZH06_ParseFrame(uint8_t *data, ZH06_Data *frame);
uint16_t ZH06CalculateChecksum(uint8_t *data, uint8_t len);
uint8_t ZH06_ReadDust(uint16_t *pm1_0, uint16_t *pm2_5, uint16_t *pm10);
void ZH06_DMAIrqHandler(void);

void ZH06RecoverDMATransfer(void);
#endif

