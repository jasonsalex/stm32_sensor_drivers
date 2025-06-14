
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
		
//		HAL_NVIC_SetPriority(USART3_4_IRQn,0,2);      //���ô���2�жϵ���ռ���ȼ�Ϊ0�������ȼ���2������Ҫע�⣬G0ϵ���ж������ȼ�������Ч
//		HAL_NVIC_EnableIRQ(USART3_4_IRQn);            //ʹ�ܴ���2���ж�
	
		u1_printf("uart sensor init...\r\n");
	
	  HAL_NVIC_SetPriority(USART3_4_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(USART3_4_IRQn);
		HAL_NVIC_SetPriority(DMA1_Ch4_7_DMAMUX1_OVR_IRQn, 0, 0);                                   //����DMA1 ͨ��2-3���жϣ����ȼ�
		HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMAMUX1_OVR_IRQn);                                           //ʹ��DMA1 ͨ��2-3���ж�

//		HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);                                   //����DMA1 ͨ��2���жϣ����ȼ�
//		HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);                                           //ʹ��DMA1 ͨ��2���ж�   
  
}

//���dma uart3_4�жϴ���
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
