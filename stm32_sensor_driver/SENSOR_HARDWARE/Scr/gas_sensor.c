
#include "gas_sensor.h"
#include "stm32g0xx_hal.h" 
#include "usart.h"
#include <math.h>

// DMA错误中断标志（对应STM32G0）
#define DMA_FLAG_TEIF DMA_ISR_TEIF1  // 传输错误中断标志
#define DMA_FLAG_FEIF DMA_ISR_FEIF1  // FIFO错误中断标志（如果支持）

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

uint8_t zp16_data_raw[9] = {0};
		
void Gas_SensorInit(void) {

	  __HAL_RCC_USART3_CLK_ENABLE();


		    // GPIO初始化（USART3 PB10/PB11）
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
    
    // UART初始化
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 9600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
		huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
		
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        //Error_Handler();
				u1_printf("hal uart3 init err....\r\n");
    }
    
    // DMA初始化
    hdma_usart3_rx.Instance = DMA1_Channel4;
    hdma_usart3_rx.Init.Request = DMA_REQUEST_USART3_RX;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
		hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH; 
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) {
        //Error_Handler();
				u1_printf("hal dma3 init err....\r\n");
    }
    __HAL_LINKDMA(&huart3, hdmarx, hdma_usart3_rx);
    __HAL_DMA_ENABLE_IT(&hdma_usart3_rx, DMA_IT_TC); // 传输完成中断
		
	  
    // 启动接收（ZP16需要9字节缓冲区）
    HAL_UART_Receive_DMA(&huart3, zp16_data_raw, sizeof(zp16_data_raw));
}


void Gas_LoopbackTest(void) {

		ZP16_Data frame;
		ZP16_ParseFrame(zp16_data_raw, &frame);
	
		if(frame.checksum == ZP16CalculateChecksum(zp16_data_raw, sizeof(zp16_data_raw)))
		{
					u1_printf("conc: %.2f mg/m3, fs：%.2f mg/m3\r\n", frame.conc, frame.fs);
		}else{
					u1_printf("read gas tvoc faild..\r\n");
		}

		memset(zp16_data_raw, 0, sizeof(zp16_data_raw));
}


void ZP16_ParseFrame(uint8_t *data, ZP16_Data *frame) {
    frame->header = data[0];
    frame->gas_type = data[1];
    frame->unit = data[2];
    frame->decimal = data[3];
    frame->conc = (float)((data[4] << 8) | data[5]) / pow(10, frame->decimal);;
    frame->fs = (float)((data[6] << 8) | data[7]) /  pow(10, frame->decimal);
    frame->checksum = data[8];
}

uint8_t ZP16_ReadTvoc(float *conc, float *fs){
		
		ZP16_Data frame;
		ZP16_ParseFrame(zp16_data_raw, &frame);
		
		*conc = frame.conc;
		*fs = frame.fs;
	
		uint8_t faild = (frame.header != 0xFF) || (frame.fs == 0.0f)|| (frame.checksum != ZP16CalculateChecksum(zp16_data_raw, sizeof(zp16_data_raw)));
		
		memset(zp16_data_raw, 0, sizeof(zp16_data_raw));
		
		if(faild)
		{
			u1_printf("zp16 rx io aboat...header:%d\r\n", frame.header);
			//ZP16RecoverDMATransfer();
		}
	
		
		return faild;
}


// 计算校验和
uint8_t ZP16CalculateChecksum(uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 1; i < len - 1; i++) {  // 字节1~7求和
        sum += data[i];
    }
    return (~sum) + 1;  // 取反加1
}


//恢复dma模式
void ZP16RecoverDMATransfer() {
	
		if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET){
					HAL_UART_IRQHandler(&huart3);
			}

}

void ZP16_DMAIrqHandler(void)
{
		uint32_t isr = huart3.Instance->ISR;
    
    if(isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) 
    {
			u1_printf("uart3 transfer, reinit..\r\n");		
			HAL_UART_DMAStop(&huart3);
			HAL_DMA_DeInit(&hdma_usart3_rx);
			Gas_SensorInit();
		}
			
		HAL_DMA_IRQHandler(&hdma_usart3_rx);  // 必须调用

}

