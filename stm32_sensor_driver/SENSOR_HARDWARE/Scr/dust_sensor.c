#include "dust_sensor.h"
#include "stm32g0xx_hal.h" 
#include <math.h>
#include "usart.h"

//// DMA错误中断标志（对应STM32G0）
//#define DMA_FLAG_TEIF DMA_ISR_TEIF1  // 传输错误中断标志
//#define DMA_FLAG_FEIF DMA_ISR_FEIF1  // FIFO错误中断标志（如果支持）

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart4_rx;

uint8_t zh06_data_raw[32] = {0};

void Dust_SensorInit(void)
{
		__HAL_RCC_USART4_CLK_ENABLE();
	
		    // GPIO初始化（USART4 PB0/PB1）
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART4;  
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
		
    // UART初始化
    huart4.Instance = USART4;
    huart4.Init.BaudRate = 9600;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
		huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart4) != HAL_OK) {
        //Error_Handler();
				u1_printf("hal uart4 init err....\r\n");
    }
    
    // DMA初始化
    hdma_usart4_rx.Instance = DMA1_Channel5;
    hdma_usart4_rx.Init.Request = DMA_REQUEST_USART4_RX;
    hdma_usart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart4_rx.Init.Mode = DMA_CIRCULAR;
		hdma_usart4_rx.Init.Priority = DMA_PRIORITY_HIGH; 
    if (HAL_DMA_Init(&hdma_usart4_rx) != HAL_OK) {
        //Error_Handler();
				u1_printf("hal dma4 init err....\r\n");
    }
		
    __HAL_LINKDMA(&huart4, hdmarx, hdma_usart4_rx);
		__HAL_DMA_ENABLE_IT(&hdma_usart4_rx, DMA_IT_TC); // 传输完成中断
      
		
    // 启动接收（ZH06需要32字节缓冲区）
    HAL_UART_Receive_DMA(&huart4, zh06_data_raw, sizeof(zh06_data_raw));

}

// 解析数据帧
void ZH06_ParseFrame(uint8_t *data, ZH06_Data *frame) {
    frame->header1 = data[0];
    frame->header2 = data[1];
    frame->frame_len = (data[2] << 8) | data[3];
    frame->pm1_0 = (data[10] << 8) | data[11];
    frame->pm2_5 = (data[12] << 8) | data[13];
    frame->pm10 = (data[14] << 8) | data[15];
    frame->checksum = (data[30] << 8) | data[31];
}

// 计算校验和 (前30字节求和)
uint16_t ZH06CalculateChecksum(uint8_t *data, uint8_t len) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len - 2; i++) {  // 字节0~29求和
        sum += data[i];
    }
    return sum;
}

// 读取粉尘浓度数据
uint8_t ZH06_ReadDust(uint16_t *pm1_0, uint16_t *pm2_5, uint16_t *pm10) {
    ZH06_Data frame;
    ZH06_ParseFrame(zh06_data_raw, &frame);
    
    // 验证帧头和校验
    uint8_t failed =(frame.header1 != 0x42) || 
                    (frame.header2 != 0x4D) ||
                    (frame.checksum != ZH06CalculateChecksum(zh06_data_raw, 32));
    
    if(!failed) {
        *pm1_0 = frame.pm1_0;
        *pm2_5 = frame.pm2_5;
        *pm10 = frame.pm10;
    }

    memset(zh06_data_raw, 0, sizeof(zh06_data_raw));
		
    if(failed) {
			  u1_printf("zh06 rx io abort...header1:%d \r\n", frame.header1);
				//ZH06RecoverDMATransfer();
    }
    
    return failed;
}


//恢复dma模式
void ZH06RecoverDMATransfer() {
    
	if(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE) != RESET){
			HAL_UART_IRQHandler(&huart4);
	}
			
		//HAL_DMA_IRQHandler(&hdma_usart4_rx);  // 必须调用
}


void ZH06_DMAIrqHandler(void)
{
		uint32_t isr = huart4.Instance->ISR;
    
   if(isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) 
    {
				u1_printf("uart4 transfer, reinit..\r\n");		
        HAL_UART_DMAStop(&huart4);
				HAL_DMA_DeInit(&hdma_usart4_rx);
				Dust_SensorInit();
    }
		
		HAL_DMA_IRQHandler(&hdma_usart4_rx);  // 必须调用
}
