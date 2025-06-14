
#include "uart_irq_sensor.h"
#include "gas_sensor.h"
#include "dust_sensor.h"
#include "stm32g0xx_hal.h" 
#include "usart.h"

void Uart_SensorsInit(void) {
   
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    
		
    Gas_SensorInit();   // ZP16
		Dust_SensorInit();  // ZH06
		
//		HAL_NVIC_SetPriority(USART3_4_IRQn,0,2);      //设置串口2中断的抢占优先级为0，子优先级是2，但是要注意，G0系列中断子优先级不会生效
//		HAL_NVIC_EnableIRQ(USART3_4_IRQn);            //使能串口2的中断
	
		u1_printf("uart sensor init...\r\n");
	
	  HAL_NVIC_SetPriority(USART3_4_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(USART3_4_IRQn);
		HAL_NVIC_SetPriority(DMA1_Ch4_7_DMAMUX1_OVR_IRQn, 0, 0);                                   //配置DMA1 通道2-3的中断，优先级
		HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMAMUX1_OVR_IRQn);                                           //使能DMA1 通道2-3的中断

//		HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);                                   //配置DMA1 通道2的中断，优先级
//		HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);                                           //使能DMA1 通道2的中断   
  
}

//检测dma uart3_4中断错误。
void USART3_4_IRQHandler(void) {
	
		ZP16RecoverDMATransfer();
		ZH06RecoverDMATransfer();
}

void DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler(void)
{
		ZP16_DMAIrqHandler();
		ZH06_DMAIrqHandler();
		//u1_printf("DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler...\r\n");
}
