
#ifndef __GAS_SENSOR_H
#define __GAS_SENSOR_H

#include <stdint.h>

/*
	���崫��������ȡ�����������Ũ�ȣ������ƾ�����������ȩ����������壬�ֱ���Ϊmg/m3
	�������ͺ�:zh16
	Э�����ͣ�uart

	Gas sensor for measuring air concentration, including alcohol, ammonia, formaldehyde, 		smoke, etc., with a resolution of mg/m3. 

	Sensor model: ZH16. 
	Protocol type: UART
	
	SDA:pb11
	SCL:pb10

*/

typedef struct {
    uint8_t header;     // 0xFF
    uint8_t gas_type;   // ��������
    uint8_t unit;       // ��λ
    uint8_t decimal;    // С��λ��
    float conc;  // Ũ��ԭʼֵ
    float fs;    // ������ԭʼֵ
    uint8_t checksum;   // У���
} ZP16_Data;



void Gas_SensorInit(void);
void Gas_LoopbackTest(void);

void ZP16_ParseFrame(uint8_t *data, ZP16_Data *frame);
uint8_t ZP16_ReadTvoc(float *conc, float *fs);
uint8_t ZP16CalculateChecksum(uint8_t *data, uint8_t len);
void ZP16_DMAIrqHandler(void);
void ZP16RecoverDMATransfer(void);

#endif 

