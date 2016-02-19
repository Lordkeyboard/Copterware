/*
  ******************************************************************************
  * @file      user_io.h
  * @author    Paul Oelsner, René Rathfelder
  * @version   V1.1
  * @date      15.02.2016
  * @brief     functions to get and set in- and output pins
  ******************************************************************************
*/

#ifndef __USER_IO_H
#define __USER_IO_H

#include "stm32f4xx.h"
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
/* #include "misc.h" */


/** @defgroup IO_defines
  * @{
  */
typedef enum
{
	led_green = (uint16_t)GPIO_Pin_12,
	led_orange = (uint16_t)GPIO_Pin_13,
	led_red = (uint16_t)GPIO_Pin_14,
	led_blue = (uint16_t)GPIO_Pin_15,
	io_button = (uint16_t)GPIO_Pin_0
} user_IO_TypeDef;

/**
  * @}
  */


/** @defgroup IO_count_defines
  * @{
  */
typedef enum
{
	led_green_count = (uint8_t)0x00,
	led_orange_count = (uint8_t)0x01,
	led_red_count = (uint8_t)0x02,
	led_blue_count = (uint8_t)0x03,
	led_max_count = (uint8_t)0x04
} user_IO_count_TypeDef;

/**
  * @}
  */

/** @defgroup status_defines
  * @{
  */
typedef enum
{
	io_button_set = (uint8_t)SET,
	io_button_reset = (uint8_t)RESET
} user_Status_TypeDef;

/**
  * @}
  */

void io_init(void);
uint8_t io_get_button(void);
void io_led_toggle(uint16_t LED);
void io_led_on(uint16_t LED);
void io_led_off(uint16_t LED);
void io_led_toggle_count_set(uint8_t LED_COUNT, uint16_t max);
void io_led_toggle_count_reset(uint8_t LED_COUNT);

#endif /* __USER_IO_H */
