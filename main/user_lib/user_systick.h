/*
  ******************************************************************************
  * @file      user_systick.h
  * @author    Paul Oelsner, René Rathfelder
  * @version   V1.4
  * @date      15.02.2016
  * @brief     functions and prototypes for initializing system timers for the STM32F4
  ******************************************************************************
  */

#ifndef __USER_SYSTICK_H
#define __USER_SYSTICK_H

/******************** Includes ********************/
#include "stm32f4xx.h" 									/* for uint definition */
#include "misc.h" 										/* IRQ functions */
#include "stm32f4xx_rcc.h" 								/* system clock functions */

/******************** Defines ********************/
#define	ticktime	1000 								/* 1ms */
/* #define	ticktime	1000000 // 1us */

/** @defgroup SysTick_IRQ_defines
  * @{
  */

typedef enum
{
	SysTick_IRQ_DISABLED = (uint8_t)0x00,
	SysTick_IRQ_ENABLED = (uint8_t)0x01
} IRQStatus_TypeDef;
/**
  * @}
  */

/******************** functions ********************/
/* static void start_tick_irq(void); */
/* static void stop_tick_irq(void); */

void SysTick_Handler(void);								/* function for control and init the SysTick */

/* function for delays and counter */
void tick_delay(uint32_t time_value);
void set_tick_counter(uint32_t time_value);
uint32_t get_tick_counter(void);
void kill_tick_counter(void);

#endif // __USER_SYSTICK_H
