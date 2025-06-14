

#ifndef __I2C_H
#define __I2C_H


#include "stm32g0xx_hal_gpio.h"


//#define SDA_GROUP             GPIOB                                               //SDA ��Ӧ��IO����
//#define SDA_PIN               GPIO_PIN_9                                          //SDA ��Ӧ��IO
#define SDA_GROUP_CLK_ENABLE  __HAL_RCC_GPIOB_CLK_ENABLE();                       //SDA ��Ӧ��IO���� ʱ��ʹ��

//#define SCL_GROUP             GPIOB                                               //SCL ��Ӧ��IO����
//#define SCL_PIN               GPIO_PIN_8                                          //SCL ��Ӧ��IO
#define SCL_GROUP_CLK_ENABLE  __HAL_RCC_GPIOB_CLK_ENABLE();                       //SCL ��Ӧ��IO���� ʱ��ʹ��
/*--------------------------------------------------------------------------*/
  		   

//#define READ_SDA        HAL_GPIO_ReadPin(SDA_GROUP, SDA_PIN)                               //��ȡSDA��ƽ 
//#define I2C_SDA_OUT(x)  HAL_GPIO_WritePin(SDA_GROUP, SDA_PIN, (GPIO_PinState)x)            //����SDA��ƽ 

//#define	I2C_SCL_H       HAL_GPIO_WritePin(SCL_GROUP, SCL_PIN, GPIO_PIN_SET)                //SCL����
//#define	I2C_SDA_H       HAL_GPIO_WritePin(SDA_GROUP, SDA_PIN, GPIO_PIN_SET)                //SDA����
 
//#define	I2C_SCL_L       HAL_GPIO_WritePin(SCL_GROUP, SCL_PIN, GPIO_PIN_RESET)              //SCL����
//#define	I2C_SDA_L       HAL_GPIO_WritePin(SDA_GROUP, SDA_PIN, GPIO_PIN_RESET)              //SDA����
 
 
 typedef struct{
	GPIO_TypeDef	*sda;
	GPIO_TypeDef	*scl;
	uint16_t sda_pin;
	uint16_t scl_pin;  
 }I2C_GPIO_REG;
 
 
void I2C_Init(I2C_GPIO_REG*);
void I2C_Start(I2C_GPIO_REG*);
void I2C_Stop(I2C_GPIO_REG*);
uint8_t I2C_Wait_Ack(I2C_GPIO_REG*);
void I2C_Send_Byte(I2C_GPIO_REG*, uint8_t);
unsigned char I2C_Read_Byte(I2C_GPIO_REG*, uint8_t);
HAL_StatusTypeDef I2C_ReadRegister(I2C_GPIO_REG* gpio,uint8_t devAddr, uint8_t regAddr, uint8_t *pData, uint8_t len);
HAL_StatusTypeDef I2C_WriteRegister(I2C_GPIO_REG* gpio, uint8_t devAddr, uint8_t regAddr, uint8_t *pData, uint8_t len);

#endif
















