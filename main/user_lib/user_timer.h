/*
  ******************************************************************************
  * @file      user_timer.h
  * @author    Paul Oelsner, René Rathfelder
  * @version   V1.3
  * @date      15.02.2016
  * @brief     functions and prototypes for timers
  ******************************************************************************
  */


#ifndef __USER_TIMER_H
#define __USER_TIMER_H

/******************** Includes ********************/
#include "stm32f4xx.h" 													/* for uint definition */
#include "misc.h" 														/* contains IRQ-functions */
#include "stm32f4xx_rcc.h" 												/* contains clock functions */
#include "stm32f4xx_tim.h" 												/* contains timer functions */

/******************** Defines ********************/
/** @defgroup TIMER_IRQ_defines
  * @{
  */

typedef enum
{
	Timer1 = (uint8_t)0x00,
	Timer2 = (uint8_t)0x01,
	Timer3 = (uint8_t)0x02,
	Timer4 = (uint8_t)0x03,
	Timer5 = (uint8_t)0x04,
	TimerEND  = (uint8_t)0x05
} TimerX_TypeDef;
/**
  * @}
  */


/** @defgroup TIMER_IRQ_Status_defines
  * @{
  */

typedef enum
{
	TimerSet = (uint8_t)0x00,
	TimerError = (uint8_t)0x01
} Timer_Status_TypeDef;
/**
  * @}
  */

typedef struct
{
	uint32_t TIM_Periph;
	uint32_t TIM_Prescale;
	TIM_TypeDef* TIMx;
	uint32_t TIM_IRQ_Channel;
} Timer_ConstTemplate_TypeDef;

/******************** functions ********************/

/* functions to execute own and independent IRQ-deactivation */
void TIMER_set_disable_flag(void);
void TIMER_reset_disable_flag(void);
uint8_t TIMER_get_disable_flag(void);

/* functions to read IRQ work status */
void TIMER_set_run_flag(void);
void TIMER_reset_run_flag(void);
uint8_t TIMER_get_run_flag(void);

/* functions to break IRQ-work status */
void TIMER_set_break_flag(void);
void TIMER_reset_break_flag(void);
uint8_t TIMER_get_break_flag(void);

/* functions for controlling and init timer */
uint8_t TIMER_init(uint8_t TimerX, uint16_t Timer_Period, uint8_t Priority, uint8_t SubPriority);
uint8_t TIMER_deinit(uint8_t TimerX);

#endif // __USER_TIMER_H
