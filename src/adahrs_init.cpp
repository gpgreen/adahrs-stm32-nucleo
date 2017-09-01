/*
 * ADAHRSInit.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#include "adahrs_init.h"
#include "stm32_dma.h"
#include "work_queue.h"

// ----------------------------------------------------------------------------

ADAHRSInit::ADAHRSInit() {
	// does nothing else
}

void ADAHRSInit::begin(void) {
	// Enable Peripheral clocks
	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1,
			ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	// assign all priority bits to preempt, none to subpriority
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	// configure LED pin
	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure pin in output push/pull mode
	GPIO_InitStructure.GPIO_Pin = LED_PIN_NUMBER;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LED_PORT_NUMBER, &GPIO_InitStructure);

	// configure DMA channels
	DMA1Channel6.begin(2, 0);
	DMA1Channel7.begin(2, 0);

	// configure Timer2 for work queue
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 2000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// Enable the TIM2 global Interrupt and set at lowest priority.
	// This is used to tell the MCU to transmit newest state data over the UART
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
			WORKQUEUE_IRQ_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// enable the timer
	TIM_Cmd(TIM2, ENABLE);
	// enable the update interrupt
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	// Start with led turned off
	led_off();

}

void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		// process the work queue
		g_work_queue.process();
		// clear pending interrupt bit
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
// ----------------------------------------------------------------------------

