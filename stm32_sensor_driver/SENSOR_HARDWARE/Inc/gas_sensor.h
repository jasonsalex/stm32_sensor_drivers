
#ifndef __GAS_SENSOR_H
#define __GAS_SENSOR_H

#include <stdint.h>

/*
	气体传感器，读取空气中气体的浓度，包含酒精，氨气，甲醛，烟雾等气体，分辨率为mg/m3
	传感器型号:zh16
	协议类型：uart

	Gas sensor for measuring air concentration, including alcohol, ammonia, formaldehyde, 		smoke, etc., with a resolution of mg/m3. 

	Sensor model: ZH16. 
	Protocol type: UART
	
	SDA:pb11
	SCL:pb10

*/

typedef struct {
    uint8_t header;     // 0xFF
    uint8_t gas_type;   // 气体类型
    uint8_t unit;       // 单位
    uint8_t decimal;    // 小数位数
    float conc;  // 浓度原始值
    float fs;    // 满量程原始值
    uint8_t checksum;   // 校验和
} ZP16_Data;



void Gas_SensorInit(void);
void Gas_LoopbackTest(void);

void ZP16_ParseFrame(uint8_t *data, ZP16_Data *frame);
uint8_t ZP16_ReadTvoc(float *conc, float *fs);
uint8_t ZP16CalculateChecksum(uint8_t *data, uint8_t len);
void ZP16_DMAIrqHandler(void);
void ZP16RecoverDMATransfer(void);

#endif 

