/*
  ******************************************************************************
  * @file      user_timer.c
  * @author    Paul Oelsner, René Rathfelder
  * @version   V1.3
  * @date      15.02.2016
  * @brief     functions and prototypes for timers
  ******************************************************************************
  */


#include "user_timer.h"

/******************** globale variables ********************/
volatile static FlagStatus global_TIMER_DISABLE_Flag = RESET; 			/* declare and initialize TIMER_DISABLE */
volatile static FlagStatus global_TIMER_RUN_Flag = RESET; 				/* global Flag to check, if IRQ ran and interrupted */
volatile static FlagStatus global_TIMER_BREAK_Flag = RESET; 			/* global Flag to message, that IRQ has been interrupted */

static const Timer_ConstTemplate_TypeDef TimerTable[TimerEND] = { //TIM_Periph,	   TIM_Prescale,									        TIMx, TIM_IRQ_Channel
																	{RCC_APB1Periph_TIM2, 60000,  /* 84.000.000 / 60.000 = 1400 Hz      */	TIM2, TIM2_IRQn}, // 32bit up-down
																	{RCC_APB1Periph_TIM5, 56000,  /* 84.000.000 / 56.000 = 1500 Hz      */	TIM5, TIM5_IRQn}, // 32bit up-down
																	{RCC_APB1Periph_TIM3, 52500,  /* 84.000.000 / 52.500 = 1600 Hz      */	TIM3, TIM3_IRQn}, // 16bit up-down
																	{RCC_APB1Periph_TIM4, 42000,  /* 84.000.000 / 42.000 = 2000 Hz      */	TIM4, TIM4_IRQn}, // 16bit up-down
																	{RCC_APB1Periph_TIM7, 31111,  /* 84.000.000 / 31.111 = 2700,00964 Hz*/	TIM7, TIM7_IRQn}  // 16bit up
																 };

/******************** WEAK functions for IRQ-Handling ********************/
void TIMER1_Update_UserIRQ_Handler(void) __attribute__((weak));
void TIMER2_Update_UserIRQ_Handler(void) __attribute__((weak));
void TIMER3_Update_UserIRQ_Handler(void) __attribute__((weak));
void TIMER4_Update_UserIRQ_Handler(void) __attribute__((weak));
void TIMER5_Update_UserIRQ_Handler(void) __attribute__((weak));

/* overwrite weak functios, if not present */
#pragma weak TIMER1_Update_UserIRQ_Handler = TIMER_Default_Update_UserIRQ_Handler
#pragma weak TIMER2_Update_UserIRQ_Handler = TIMER_Default_Update_UserIRQ_Handler
#pragma weak TIMER3_Update_UserIRQ_Handler = TIMER_Default_Update_UserIRQ_Handler
#pragma weak TIMER4_Update_UserIRQ_Handler = TIMER_Default_Update_UserIRQ_Handler
#pragma weak TIMER5_Update_UserIRQ_Handler = TIMER_Default_Update_UserIRQ_Handler

/* function to overwrite handler, if non existent */
static void TIMER_Default_Update_UserIRQ_Handler(void)
{
	//do nothing
}

/******************** functions ********************/

/* function to set automatic self-deinit of all timers */
void TIMER_set_disable_flag(void)
{
	global_TIMER_DISABLE_Flag = SET;
}

/* function to reset all automatic self-deinit of all timers */
void TIMER_reset_disable_flag(void)
{
	global_TIMER_DISABLE_Flag = RESET;
}

/* function to get automatic self-deinit flag of all timers */
uint8_t TIMER_get_disable_flag(void)
{
	return global_TIMER_DISABLE_Flag;
}

/* function to set the execution of timer */
void TIMER_set_run_flag(void)
{
	global_TIMER_RUN_Flag = SET;
}

/* functionto reset the execution of timer */
void TIMER_reset_run_flag(void)
{
	global_TIMER_RUN_Flag = RESET;
}

/* function to get execution flag of timer */
uint8_t TIMER_get_run_flag(void)
{
	return global_TIMER_RUN_Flag;
}

/* function to set execution of state machine?*/
void TIMER_set_break_flag(void)
{
	global_TIMER_BREAK_Flag = SET;
}

/* function to reset execution of state machine */
void TIMER_reset_break_flag(void)
{
	global_TIMER_BREAK_Flag = RESET;
}

/* function to get execution of state machine */
uint8_t TIMER_get_break_flag(void)
{
	return global_TIMER_BREAK_Flag;
}

/* function to init a timer with given frequency and activate interrupt */
uint8_t TIMER_init(uint8_t TimerX, uint16_t Timer_Period, uint8_t Priority, uint8_t SubPriority)
{

	uint16_t Period = 1; 													/* init with max Hz */
	uint8_t return_value = TimerError;

	if(Timer_Period != 0)
	{
		Period = Timer_Period;
	}

	if(TimerX < TimerEND)
	{
		return_value = TimerSet; 											/* set Timer Status -> okay */

		RCC_APB1PeriphClockCmd(TimerTable[TimerX].TIM_Periph, ENABLE); 		/* connect clock with timer */

		TIM_TimeBaseInitTypeDef TIM_Struct;
		TIM_Struct.TIM_Prescaler = TimerTable[TimerX].TIM_Prescale - 1; 	/* set clock 84.000.000 / 420.000 = 200 Hz */
		TIM_Struct.TIM_CounterMode = TIM_CounterMode_Up; 					/* Count-up timer mode */
		TIM_Struct.TIM_Period = Period - 1; 								/* 200 Hz to 1 Hz = 1 second */
		TIM_Struct.TIM_ClockDivision = TIM_CKD_DIV1; 						/* divide by 1 */
		TIM_Struct.TIM_RepetitionCounter = 0; 								/* don't use rep counter */
		TIM_TimeBaseInit(TimerTable[TimerX].TIMx, &TIM_Struct); 			/* init timer */

		TIM_ITConfig(TimerTable[TimerX].TIMx, TIM_IT_Update, ENABLE); 		/* activate update interrupt flag */

		/* set interrupt settings */
		NVIC_InitTypeDef TIM_IRQ_Struct; 									/* struct for IRQ timer settings */
		TIM_IRQ_Struct.NVIC_IRQChannel = TimerTable[TimerX].TIM_IRQ_Channel;/* set IRQ-channels for UART2 -> see stm32f4xx.h */
		TIM_IRQ_Struct.NVIC_IRQChannelPreemptionPriority = Priority; 		/* set USART-priority */
		TIM_IRQ_Struct.NVIC_IRQChannelSubPriority = SubPriority; 			/* set USART2-priotity */
		TIM_IRQ_Struct.NVIC_IRQChannelCmd = ENABLE; 						/* activate global IRQ-Channel */
		NVIC_Init(&TIM_IRQ_Struct); 										/* init IRQ with given settings */

		TIM_Cmd(TimerTable[TimerX].TIMx, ENABLE); 							/* activate timer */
	}

	return return_value;
}


/* function to deinit a Timers and load default settings and deactivate Interrupt */
uint8_t TIMER_deinit(uint8_t TimerX)
{
	uint8_t return_value = TimerError;

	if(TimerX < TimerEND)
	{
		return_value = TimerSet; 											/* set timer Status -> okay */
		RCC_APB1PeriphClockCmd(TimerTable[TimerX].TIM_Periph, DISABLE); 	/* disconnect clock and timer */
		TIM_TimeBaseInitTypeDef TIM_Struct;
		TIM_TimeBaseStructInit(&TIM_Struct); 								/* load default settings */
		TIM_TimeBaseInit(TimerTable[TimerX].TIMx, &TIM_Struct); 			/* init timer */
		TIM_ITConfig(TimerTable[TimerX].TIMx, TIM_IT_Update, DISABLE); 		/* deactivate update-Interrupt Flag */
		NVIC_DisableIRQ(TimerTable[TimerX].TIM_IRQ_Channel); 				/* disable timer-Interrupts in IRQ-Controller */
		TIM_Cmd(TimerTable[TimerX].TIMx, DISABLE); 							/* deactivate timer */
	}

	return return_value;
}


/******************** private functions ********************/


/******************** IRQ-Handler functions ********************/

void TIM2_IRQHandler(void)													/* function which gets called by ISR from TIM2 */
{																				/* check and reset of Interrupt flag */
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)								/* for naming see startup_stm32f4xx.c */
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 						/* reset flag */
		TIMER1_Update_UserIRQ_Handler();									/* execute own handler */
	}
}

void TIM3_IRQHandler(void)													/* function which gets called by ISR from TIM3 */
{																				/* check and reset of Interrupt flag */
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)								/* for naming see startup_stm32f4xx.c */
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update); 						/* reset flag */
		TIMER3_Update_UserIRQ_Handler();									/* execute own handler */
	}
}

void TIM4_IRQHandler(void)													/* function which gets called by ISR from TIM4 */
{																				/* check and reset of Interrupt flag */
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)								/* for naming see startup_stm32f4xx.c */
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update); 						/* reset flag */
		TIMER4_Update_UserIRQ_Handler();									/* execute own handler */
	}
}

void TIM5_IRQHandler(void)													/* function which gets called by ISR from TIM5 */
{																				/* check and reset of Interrupt flag */
	if(TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)								/* for naming see startup_stm32f4xx.c */
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update); 						/* reset flag */
		TIMER2_Update_UserIRQ_Handler();									/* execute own handler */
	}
}

void TIM7_IRQHandler(void)													/* function which gets called by ISR from TIM7 */
{																				/* check and reset of Interrupt flag */
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)								/* for naming see startup_stm32f4xx.c */
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update); 						/* reset flag */
		TIMER5_Update_UserIRQ_Handler();									/* execute own handler */
	}
}
