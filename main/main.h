/*
  ******************************************************************************
  * @file      main.h
  * @author    René Rathfelder
  * @version   V1.1
  * @date      15.02.2016
  * @brief     main function
  * 		   Hint: "cmsis_boot/stm32f4xx.h", "cmsis_boot/system_stm32f4xx.c" have to be set at 8MHz!
  ******************************************************************************
  */

#ifndef __STM32F4_MAIN_H
#define __STM32F4_MAIN_H

/******************** Includes ********************/
//#include <stdlib.h> 													/* for malloc function */
#include "misc.h" 														/* for Interrupt priorities */
/* #include "user_usart.h" 											*/	/* user lib usart2 */
/* #include "user_can1.h" 											*/	/* user lib can1 */
#include "user_io.h" 													/* user lib for GPIOs */
#include "user_systick.h" 												/* counter and timer */
#include "user_automat.h" 												/* state machine functions */

/******************** Defines ********************/

/******************** Funktionen ********************/

#endif // __STM32F4_MAIN_H
