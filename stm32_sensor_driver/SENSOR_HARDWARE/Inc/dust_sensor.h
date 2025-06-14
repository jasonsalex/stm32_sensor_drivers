#ifndef __DUST_SENSOR_H
#define __DUST_SENSOR_H

#include <stdint.h>

/*
	空气质量传感器，检测空中的粉尘含量，检测pm1.0,pm2.5,pm10，分辨率为μg/m3..
	传感器型号:zh06
	协议类型：uart

	Air quality sensor for measuring airborne dust concentration, including PM1.0, PM2.5, 		and PM10, with a resolution of μg/m3
	
	Sensor model: ZH06 
	Protocol type: UART

	SDA:pb11
	SCL:pb10
*/

// ZH06 数据结构体
typedef struct {
    uint8_t header1;      // 起始字节1 (0x42)
    uint8_t header2;      // 起始字节2 (0x4D)
    uint16_t frame_len;   // 帧长度
    uint16_t pm1_0;       // PM1.0浓度 (μg/m3)
    uint16_t pm2_5;       // PM2.5浓度 (μg/m3)
    uint16_t pm10;        // PM10浓度 (μg/m3)
    uint16_t checksum;    // 校验值 (高8位+低8位)
} ZH06_Data;

void Dust_SensorInit(void);
void ZH06_ParseFrame(uint8_t *data, ZH06_Data *frame);
uint16_t ZH06CalculateChecksum(uint8_t *data, uint8_t len);
uint8_t ZH06_ReadDust(uint16_t *pm1_0, uint16_t *pm2_5, uint16_t *pm10);
void ZH06_DMAIrqHandler(void);

void ZH06RecoverDMATransfer(void);
#endif

