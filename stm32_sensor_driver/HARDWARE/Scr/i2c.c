#include "stm32g0xx_hal.h"    //������Ҫ��ͷ�ļ�
#include "clock.h"            //������Ҫ��ͷ�ļ�
#include "i2c.h"              //������Ҫ��ͷ�ļ�

void I2C_Init(I2C_GPIO_REG* gpio) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    SDA_GROUP_CLK_ENABLE;
    SCL_GROUP_CLK_ENABLE;

    // SDA ����Ϊ��©�����������ʱ���ⲿ�������裩
    GPIO_InitStruct.Pin = gpio->sda_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(gpio->sda, &GPIO_InitStruct);

    // SCL ����Ϊ�����������ǿ����������
    GPIO_InitStruct.Pin = gpio->scl_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(gpio->scl, &GPIO_InitStruct);
		

    // ��ʼ״̬������ SCL �� SDA
    HAL_GPIO_WritePin(gpio->scl, gpio->scl_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(gpio->sda, gpio->sda_pin, GPIO_PIN_SET);
		
}

void I2C_Start(I2C_GPIO_REG* gpio) {
    HAL_GPIO_WritePin(gpio->sda, gpio->sda_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(gpio->scl, gpio->scl_pin, GPIO_PIN_SET);
    Delay_Us(5);
    HAL_GPIO_WritePin(gpio->sda, gpio->sda_pin, GPIO_PIN_RESET);
    Delay_Us(5);
    HAL_GPIO_WritePin(gpio->scl, gpio->scl_pin, GPIO_PIN_RESET);
}

void I2C_Stop(I2C_GPIO_REG* gpio) {
    HAL_GPIO_WritePin(gpio->sda, gpio->sda_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(gpio->scl, gpio->scl_pin, GPIO_PIN_SET);
    Delay_Us(5);
    HAL_GPIO_WritePin(gpio->sda, gpio->sda_pin, GPIO_PIN_SET);
    Delay_Us(5);
}

uint8_t I2C_Wait_Ack(I2C_GPIO_REG* gpio) {
    uint32_t timeout = 1000; // ��ʱ����

    // �л� SDA Ϊ����ģʽ
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = gpio->sda_pin,
        .Mode = GPIO_MODE_INPUT,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };
    HAL_GPIO_Init(gpio->sda, &GPIO_InitStruct);

    HAL_GPIO_WritePin(gpio->scl, gpio->scl_pin, GPIO_PIN_SET);
    Delay_Us(2);

    while (HAL_GPIO_ReadPin(gpio->sda, gpio->sda_pin)) {
        if (--timeout == 0) {
            I2C_Stop(gpio);
            return 1; // ��ʱ��Ӧ��
        }
        Delay_Us(10);
    }

    HAL_GPIO_WritePin(gpio->scl, gpio->scl_pin, GPIO_PIN_RESET);
    
    // �ָ� SDA Ϊ��©���
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(gpio->sda, &GPIO_InitStruct);

    return 0; // �ɹ�Ӧ��
}

void I2C_Send_Byte(I2C_GPIO_REG* gpio, uint8_t txd) {
    for (uint8_t i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(gpio->sda, gpio->sda_pin, (txd & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        txd <<= 1;
        Delay_Us(2);
        HAL_GPIO_WritePin(gpio->scl, gpio->scl_pin, GPIO_PIN_SET);
        Delay_Us(5);
        HAL_GPIO_WritePin(gpio->scl, gpio->scl_pin, GPIO_PIN_RESET);
        Delay_Us(2);
    }
}

uint8_t I2C_Read_Byte(I2C_GPIO_REG* gpio, uint8_t ack) {
    uint8_t receive = 0;

    // �л� SDA Ϊ����ģʽ
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = gpio->sda_pin,
        .Mode = GPIO_MODE_INPUT,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };
    HAL_GPIO_Init(gpio->sda, &GPIO_InitStruct);

    for (uint8_t i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(gpio->scl, gpio->scl_pin, GPIO_PIN_SET);
        Delay_Us(5);
        receive <<= 1;
        if (HAL_GPIO_ReadPin(gpio->sda, gpio->sda_pin)) receive |= 0x01;
        HAL_GPIO_WritePin(gpio->scl, gpio->scl_pin, GPIO_PIN_RESET);
        Delay_Us(5);
    }

    // �ָ� SDA Ϊ��©���
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(gpio->sda, &GPIO_InitStruct);

    // ���� ACK/NACK
    HAL_GPIO_WritePin(gpio->sda, gpio->sda_pin, ack ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(gpio->scl, gpio->scl_pin, GPIO_PIN_SET);
    Delay_Us(5);
    HAL_GPIO_WritePin(gpio->scl, gpio->scl_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(gpio->sda, gpio->sda_pin, GPIO_PIN_SET);

    return receive;
}

HAL_StatusTypeDef I2C_WriteRegister(I2C_GPIO_REG* gpio, uint8_t devAddr, uint8_t regAddr, uint8_t* pData, uint8_t len) {
    I2C_Start(gpio);
	
    I2C_Send_Byte(gpio, devAddr);

    if (I2C_Wait_Ack(gpio)) return HAL_ERROR;

    I2C_Send_Byte(gpio, regAddr);
    if (I2C_Wait_Ack(gpio)) return HAL_ERROR;

    for (uint8_t i = 0; i < len; i++) {
        I2C_Send_Byte(gpio, pData[i]);
        if (I2C_Wait_Ack(gpio)) return HAL_ERROR;
    }

    I2C_Stop(gpio);
    return HAL_OK;
}

HAL_StatusTypeDef I2C_ReadRegister(I2C_GPIO_REG* gpio, uint8_t devAddr, uint8_t regAddr, uint8_t* pData, uint8_t len) {
    I2C_Start(gpio);
	
    I2C_Send_Byte(gpio, devAddr);
    if (I2C_Wait_Ack(gpio)) return HAL_ERROR;

    I2C_Send_Byte(gpio, regAddr);
    if (I2C_Wait_Ack(gpio)) return HAL_ERROR;

    I2C_Start(gpio);
    I2C_Send_Byte(gpio, (devAddr) | 0x01);

    if (I2C_Wait_Ack(gpio)) return HAL_ERROR;

    for (uint8_t i = 0; i < len; i++) {
        pData[i] = I2C_Read_Byte(gpio, (i == len - 1) ? 0 : 1);
    }

    I2C_Stop(gpio);
    return HAL_OK;
}
