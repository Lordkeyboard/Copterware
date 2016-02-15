/*
  ******************************************************************************
  * @file      user_systick.c
  * @author    Paul Oelsner, René Rathfelder
  * @version   V1.3
  * @date      15.02.2016
  * @brief	   functions and prototypes for initializing system timers for the STM32F4
  ******************************************************************************
  */

#include "user_systick.h"

/******************** global variables ********************/
volatile static uint32_t user_counter = 0;
volatile static uint32_t user_delay = 0;
volatile static uint8_t irq_status = SysTick_IRQ_DISABLED;

/******************** functions ********************/

/* function to initialize a systick with interrupt. The priority is always the lowest independent of chosen group.*/
static void start_tick_irq(void)
{
	RCC_ClocksTypeDef RCC_Clocks; 									/* rcc struct erstellen */
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / ticktime); 			/* jede 1ms auslösen */

	/* Interrupt setting */
	NVIC_InitTypeDef SysTick_IRQ_Struct; 							/* struct to set the Systick IRQ */
	SysTick_IRQ_Struct.NVIC_IRQChannel = SysTick_IRQn; 				/* IRQ-channels for systick -> see stm32f4xx.h */
	SysTick_IRQ_Struct.NVIC_IRQChannelPreemptionPriority = 15; 		/* systick priority to very low */
	SysTick_IRQ_Struct.NVIC_IRQChannelSubPriority = 15; 			/* systick priority to very low */
	SysTick_IRQ_Struct.NVIC_IRQChannelCmd = ENABLE; 				/* activate global IRQ Channel */
	NVIC_Init(&SysTick_IRQ_Struct); 								/* init IRQ with given settings */
	irq_status = SysTick_IRQ_ENABLED;
}

static void stop_tick_irq(void)										/* function to deinit a systicks with interrupt. */
{
	NVIC_DisableIRQ(SysTick_IRQn); 									/* disable systick interrupts in IRQ-Controller */
	irq_status = SysTick_IRQ_DISABLED;
}


void tick_delay(uint32_t time_value)								/* waiting until given time is up */
{
	if(irq_status == SysTick_IRQ_DISABLED) 							/* if irq not activated */
	{
		start_tick_irq();
	}

	user_delay = time_value; 										/* set delay in ms */

	while(user_delay != 0)
	{
																	/* wait until delay is up */
	}
}

void set_tick_counter(uint32_t time_value)							/* function to set a counter */
{
	if(irq_status == SysTick_IRQ_DISABLED) 							/* if irq not activated */
	{
		start_tick_irq();
	}

	user_counter = time_value; 										/* set counter */
}

uint32_t get_tick_counter(void)										/* function to read a counter */
{
	return user_counter;
}


void kill_tick_counter(void)										/* function to reset a counter */
{
	user_counter = 0;
}

void SysTick_Handler(void)											/* function which is called by Systick ISR */
{
	NVIC_DisableIRQ(SysTick_IRQn); 									/* disable systick interrupts in IRQ Controller */
	uint8_t status = 0;

	if(user_delay != 0)  											/* as long as delay not zero */
	{
		user_delay--;
		status = 1;
	}

	if(user_counter != 0)  											/* as long as counter not zero */
	{
		user_counter--;
		status = 1;
	}

	if(status == 0) 												/* if no active counter */
	{
		stop_tick_irq(); 											/* stop IRQ */
	}
	else
	{
		NVIC_EnableIRQ(SysTick_IRQn);  								/* enable systick Interrupt in IRQ Controller */
	}
}
