#include "light_sensor.h"
#include "stm32g0xx_hal.h"  //������Ҫ��ͷ�ļ�
//#include "i2c_hard.h"
#include "clock.h" 	        //������Ҫ��ͷ�ļ�			 
#include "usart.h"          //������Ҫ��ͷ�ļ�
#include "i2c.h" 

#define LTR390_ADDR         0x53 << 1
#define LTR390_PART_ID      0x06
#define LTR390_MAIN_CTRL    0x00
#define LTR390_MEAS_RATE    0x04
#define LTR390_GAIN         0x05
#define LTR390_ALSDATA      0x0D
#define LTR390_UVSDATA      0x10

static I2C_GPIO_REG gpio = {
    .scl = GPIOB,
    .sda = GPIOB,
    .scl_pin = GPIO_PIN_3,
    .sda_pin = GPIO_PIN_4
};


void Test_UVS_With_Debug() {
    if (LTR390_Init() != 0) {
        u1_printf("Sensor Init Failed!\r\n");
        return;
    }

    uint32_t uv_value;
    while (1) {
        // 1. �л��� UVS ģʽ
        uint8_t mode = 0x0A;
        if (I2C_WriteRegister(&gpio, LTR390_ADDR, LTR390_MAIN_CTRL, &mode, 1) != HAL_OK) {
            u1_printf("Set UVS Mode Failed!\r\n");
            break;
        }

        // 2. �ض� MAIN_CTRL
        uint8_t current_mode;
        if (I2C_ReadRegister(&gpio, LTR390_ADDR, LTR390_MAIN_CTRL, &current_mode, 1) != HAL_OK) {
            u1_printf("Read MAIN_CTRL Failed!\r\n");
            break;
        }
        u1_printf("MAIN_CTRL: 0x%02X\r\n", current_mode);

        // 3. ��ȡ UVS ����
        uint8_t buf[3];
        if (I2C_ReadRegister(&gpio, LTR390_ADDR, LTR390_UVSDATA, buf, 3) != HAL_OK) {
            u1_printf("Read UVSDATA Failed!\r\n");
            break;
        }
        uv_value = (buf[2] << 16) | (buf[1] << 8) | buf[0];
        u1_printf("UVS RAW: 0x%02X 0x%02X 0x%02X | Value: %lu\r\n", buf[0], buf[1], buf[2], uv_value);

        HAL_Delay(1000);
    }
}

uint8_t LTR390_Init(void) {
    I2C_Init(&gpio);

    // 1. �ϵ粢��� MAIN_CTRL
    uint8_t ctrl_val = 0x01; // �ϵ磨Bit0=1��
    if (I2C_WriteRegister(&gpio, LTR390_ADDR, LTR390_MAIN_CTRL, &ctrl_val, 1) != HAL_OK) {
        u1_printf("Error: LTR390 Failed to power on!\r\n");
        return 1;
    }
    HAL_Delay(50);

    // 2. �ض� MAIN_CTRL ��֤
    if (I2C_ReadRegister(&gpio, LTR390_ADDR, LTR390_MAIN_CTRL, &ctrl_val, 1) != HAL_OK) {
        u1_printf("Error: LTR390 Failed to read MAIN_CTRL!\r\n");
        return 1;
    }
    u1_printf("MAIN_CTRL = 0x%02X\r\n", ctrl_val); // Ԥ�� 0x01

    // 3. ��֤���� ID
    uint8_t id;
    if (I2C_ReadRegister(&gpio, LTR390_ADDR, LTR390_PART_ID, &id, 1) != HAL_OK) {
        u1_printf("Error: LTR390 Failed to read PART_ID!\r\n");
        return 1;
    }
    u1_printf("PART_ID = 0x%02X\r\n", id); // Ԥ�� 0xB2
    if (id != 0xB2) return 1;

    // 4. ���ò�������
    uint8_t meas_rate = 0x02; // 18-bit, 100ms
    uint8_t gain = 0x02;      // 6x
    if (I2C_WriteRegister(&gpio, LTR390_ADDR, LTR390_MEAS_RATE, &meas_rate, 1) != HAL_OK ||
        I2C_WriteRegister(&gpio, LTR390_ADDR, LTR390_GAIN, &gain, 1) != HAL_OK) {
        u1_printf("Error: Failed to configure sensor!\r\n");
        return 1;
    }

    u1_printf("LTR390 Init OK!\r\n");
    return 0;
}

float LTR390_Calculate_Lux(uint32_t raw_als, uint8_t gain, uint8_t integ_time) {
    // ������ձ�
    const float gain_table[] = {1.0f, 3.0f, 6.0f, 9.0f, 18.0f};
    // ����ʱ����ձ���λ��ms��
    const float time_table[] = {25.0f, 50.0f, 100.0f, 200.0f, 500.0f, 1000.0f, 2000.0f};
    
    float lux = (raw_als * gain_table[gain] * time_table[integ_time]) / 912.0f;
    return lux;
}

uint8_t LTR390_ReadAls(float* als_data) {
    uint8_t buf[3];
    uint8_t mode = 0x02; // ALS ģʽ
	  uint8_t gain, meas_rate;

    //HAL_Delay(100); // �ȴ������ȶ�

    // �л��� ALS ģʽ
    if (I2C_WriteRegister(&gpio, LTR390_ADDR, LTR390_MAIN_CTRL, &mode, 1) != HAL_OK) {
        u1_printf("Error: LTR390 Failed to set ALS mode!\r\n");
				LTR390_Init();
        return 1;
    }
		
		
		HAL_Delay(500); // �ȴ������ȶ�

    // ��ȡ ALS ����
    if (I2C_ReadRegister(&gpio, LTR390_ADDR, LTR390_ALSDATA, buf, 3) != HAL_OK) {
        u1_printf("Error: LTR390 Failed to read ALS data!\n");
        return 1;
    }
		
    uint32_t raw_als = (buf[2] << 16) | (buf[1] << 8) | buf[0];
				   
		
    I2C_ReadRegister(&gpio, LTR390_ADDR, 0x05, &gain, 1);
    I2C_ReadRegister(&gpio, LTR390_ADDR, 0x04, &meas_rate, 1);
    gain &= 0x07;       // ȡ��3λ
    meas_rate &= 0x07;  // ȡ��3λ
		
		*als_data = LTR390_Calculate_Lux(raw_als, gain, meas_rate);
		
    return 0;
}

uint8_t LTR390_ReadUvs(float* uvs_data){
				uint8_t buf[3];
	      uint8_t mode = 0x0A;
	
        if (I2C_WriteRegister(&gpio, LTR390_ADDR, LTR390_MAIN_CTRL, &mode, 1) != HAL_OK) {
            u1_printf("Set LTR390 UVS Mode Failed!\r\n");
						LTR390_Init();
						return 1;
        }
				
				HAL_Delay(500); // �ȴ������ȶ�
			
        if (I2C_ReadRegister(&gpio, LTR390_ADDR, LTR390_UVSDATA, buf, 3) != HAL_OK) {
            u1_printf("Read LTR390 UVSDATA Failed!\r\n");
						return 1;
        }
//				uint32_t raw_uvs = ((buf[2] & 0x0F) << 16) | (buf[1] << 8) | buf[0];
//				
//        *uvs_data = raw_uvs * (0.01f /1000.0f);
				uint32_t raw_uvs = ((buf[2] & 0x03) << 16) | (buf[1] << 8) | buf[0];

					// ת��Ϊ UV ǿ�ȣ���W/cm2�������� LSB_SIZE = 0.01 ��W/cm2
				*uvs_data = raw_uvs * 0.01f;
				
				return 0;
}







