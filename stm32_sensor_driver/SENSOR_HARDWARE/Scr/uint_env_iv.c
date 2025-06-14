#include "stm32g0xx_hal.h"  //包含需要的头文件
#include "uint_env_iv.h"         //包含需要的头文件
#include "clock.h" 	        //包含需要的头文件			 
#include "usart.h"          //包含需要的头文件
#include "i2c.h"


#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_XLSB  0xF9
#define BMP280_REG_CALIB_P1    0x8E  // 气压校准起始地址
#define BMP280_ADDR_WRITE 0xEC
#define BMP280_ADDR_READ  0xED
#define BMP280_ADDR 0x76 << 1

static I2C_GPIO_REG gpio = {
    .scl = GPIOB,
    .sda = GPIOB,
    .scl_pin = GPIO_PIN_14,
    .sda_pin = GPIO_PIN_13,
};


void EnvIV_SensorInit(void)
{	
	I2C_Init(&gpio);
	BMP280_CalibInit(); // 只需初始化一次
}


void SHT40_Write(uint8_t command) {
    I2C_Start(&gpio);                        // 起始信号
    I2C_Send_Byte(&gpio,	0x88);                // SHT40写地址（0x44 << 1 | 0x00）
    if (I2C_Wait_Ack(&gpio)) {
        u1_printf("SHT40写地址无应答!\r\n");
        I2C_Stop(&gpio);
        return;
    }
    I2C_Send_Byte(&gpio,command);             // 发送测量命令（如0xFD为高精度模式）
    if (I2C_Wait_Ack(&gpio)) {
        u1_printf("SHT40命令无应答!\r\n");
    }
    I2C_Stop(&gpio);                         // 停止信号
}


uint8_t SHT40_ReadTempHum(float *temp_ptr, float *hum_ptr) {
    // 1. 触发高精度测量
    SHT40_Write(0xFD);  // 测量命令
    HAL_Delay(10);      // 等待测量完成

    // 2. 读取6字节数据
    uint8_t data[6];
    I2C_Start(&gpio);
		I2C_Send_Byte(&gpio,0x89); 
		if(I2C_Wait_Ack(&gpio)) {  // 读地址
        I2C_Stop(&gpio);
        return 1;  // 通信错误
    }
    
    for (int i = 0; i < 5; i++) {
        data[i] = I2C_Read_Byte(&gpio,1);  // ACK
    }
    data[5] = I2C_Read_Byte(&gpio,0);      // NACK
    I2C_Stop(&gpio);

    // 3. CRC校验（可选但推荐）
    if (CheckCRC8(&data[0], 2) != data[2] || 
        CheckCRC8(&data[3], 2) != data[5]) {
        return 2;  // CRC错误
    }

    // 4. 计算温湿度
    uint16_t raw_temp = (data[0] << 8) | data[1];
    uint16_t raw_hum = (data[3] << 8) | data[4];
    
    *temp_ptr = -45.0f + 175.0f * (raw_temp / 65535.0f);  // 温度公式
    *hum_ptr = -6.0f + 125.0f * (raw_hum / 65535.0f);     // 湿度公式
    
    return 0;  // 成功
}



void SHT40_StartMeasurement(void) {
    SHT40_Write(0xFD);                  // 0xFD：高精度测量命令
    HAL_Delay(10);                      // 等待测量完成（高精度模式约8.2ms）
}

void SHT40_Sleep() // 测量后进入休眠
{
	SHT40_Write(0xB0);  // SHT40低功耗命令
}

static BMP280_Calib bmp280_cal;

void BMP280_CalibInit(void) {
    uint8_t calib_data[24];
    // 读取全部24字节校准参数（0x88~0x9F）
    if (I2C_ReadRegister(&gpio, BMP280_ADDR, 0x88, calib_data, 24) != HAL_OK) {
        u1_printf("BMP280 calibration read failed!\r\n");
        return;
    }

    // 解析校准参数（注意符号扩展！）
    bmp280_cal.dig_T1 = (uint16_t)((calib_data[1] << 8) | calib_data[0]);
    bmp280_cal.dig_T2 = (int16_t)((calib_data[3] << 8) | calib_data[2]);
    bmp280_cal.dig_T3 = (int16_t)((calib_data[5] << 8) | calib_data[4]);

    bmp280_cal.dig_P1 = (uint16_t)((calib_data[7] << 8) | calib_data[6]);
    bmp280_cal.dig_P2 = (int16_t)((calib_data[9] << 8) | calib_data[8]);
    bmp280_cal.dig_P3 = (int16_t)((calib_data[11] << 8) | calib_data[10]);
    bmp280_cal.dig_P4 = (int16_t)((calib_data[13] << 8) | calib_data[12]);
    bmp280_cal.dig_P5 = (int16_t)((calib_data[15] << 8) | calib_data[14]);
    bmp280_cal.dig_P6 = (int16_t)((calib_data[17] << 8) | calib_data[16]);
    bmp280_cal.dig_P7 = (int16_t)((calib_data[19] << 8) | calib_data[18]);
    bmp280_cal.dig_P8 = (int16_t)((calib_data[21] << 8) | calib_data[20]);
    bmp280_cal.dig_P9 = (int16_t)((calib_data[23] << 8) | calib_data[22]);
		HAL_Delay(5);
}

float BMP280_ReadPressureWithCompensation(void) {
	
		if (!BMP280_Detect()) {
				u1_printf("BMP280 discoenect! retry...\r\n");
				BMP280_CalibInit();
				HAL_Delay(100);
				return 0;
		}
		
		    // 1. 触发测量（强制模式）
    uint8_t ctrl_meas = 0x13; // osrs_t=001, osrs_p=010, mode=01（强制模式）
    if (I2C_WriteRegister(&gpio, BMP280_ADDR, 0xF4, &ctrl_meas, 1) != HAL_OK) {
        u1_printf("BMP280 trigger failed!\r\n");
        return 0;
    }
    HAL_Delay(10); // 等待测量完成

    // 2. 读取原始数据
    uint8_t data[6];
    if (I2C_ReadRegister(&gpio, BMP280_ADDR, 0xF7, data, 6) != HAL_OK) {
        u1_printf("BMP280 data read failed!\r\n");
        return 0;
    }
		

    // 3. 计算补偿温度
    int32_t adc_T = (data[3]<<12) | (data[4]<<4) | (data[5]>>4);
    int32_t var1 = (((adc_T>>3) - (bmp280_cal.dig_T1<<1)) * bmp280_cal.dig_T2) >> 11;
    int32_t var2 = (((((adc_T>>4) - bmp280_cal.dig_T1) * ((adc_T>>4) - bmp280_cal.dig_T1)) >> 12) * bmp280_cal.dig_T3) >> 14;
    int32_t t_fine = var1 + var2;

    // 4. 计算补偿气压
    int32_t adc_P = (data[0]<<12) | (data[1]<<4) | (data[2]>>4);
		
    int64_t var1_p = t_fine - 128000;
    int64_t var2_p = var1_p * var1_p * bmp280_cal.dig_P6;
    var2_p += (var1_p * bmp280_cal.dig_P5) << 17;
    var2_p += ((int64_t)bmp280_cal.dig_P4) << 35;
    var1_p = ((var1_p * var1_p * bmp280_cal.dig_P3)>>8) + ((var1_p * bmp280_cal.dig_P2)<<12);
    var1_p = ((((int64_t)1)<<47) + var1_p) * bmp280_cal.dig_P1 >> 33;
    
		if(var1_p == 0) //检测是否拿到气压数据
		{
			BMP280_CalibInit(); // 重新初始化一次
			return 0;
		}

    int64_t p = 1048576 - adc_P;
    p = (((p<<31) - var2_p) * 3125) / var1_p;
    var1_p = (bmp280_cal.dig_P9 * (p>>13) * (p>>13)) >> 25;
    var2_p = (bmp280_cal.dig_P8 * p) >> 19;
    p = ((p + var1_p + var2_p) >> 8) + ((int64_t)bmp280_cal.dig_P7 << 4);
		
    return (float)p / 25600.0f;; // 转换为hPa
}


// 气压传感器检测函数
uint8_t BMP280_Detect(void) {
    uint8_t id = 0;
    if (I2C_ReadRegister(&gpio, BMP280_ADDR_WRITE, 0xD0, &id, 1) == HAL_OK) {
        return (id == 0x58); // BMP280的ID应为0x58
    }else{
				u1_printf("BMP280 detect flag error\r\n");
		}
    return 0;
}

//进行传感器数据crc校验
uint8_t CheckCRC8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x31;
            else crc <<= 1;
        }
    }
    return crc;
}




