/*
  ******************************************************************************
  * @file      user_usart.h
  * @author    Paul Oelsner
  * @version   V1.9
  * @date      15.10.2014
  * @brief     functions and prototypes for init and control USART on STM32F4107
  ******************************************************************************
  */


#ifndef __USER_USART_H
#define __USER_USART_H

/******************** Includes ********************/
#include "stm32f4xx.h"														/* for uint definition */
#include "misc.h" 															/* for IRQ-functions */
#include "stm32f4xx_usart.h" 												/* USART functions */
#include "stm32f4xx_gpio.h" 												/* GPIO functions */
#include "stm32f4xx_rcc.h" 													/* bus struct functions */
#include "user_systick.h" 													/* wait functions */

/******************** Defines ********************/
/**
 * @brief  Wait till USART finishes transmission
 */
#define USART_WAIT(USARTx)                  do { while (!((USARTx)->SR & USART_FLAG_TXE)); } while (0)



/** @defgroup USART2IRQ-Flags_defines
  * @{
  */
typedef enum
{
	user_USART_CTS = (uint16_t)USART_IT_CTS, 								/* CTS IRQ-Flag */
	user_USART_RXNE = (uint16_t)USART_IT_RXNE 								/* RXNE IRQ-Flag */
} user_USART_IRQ_TypeDef;

/**
  * @}
  */

/** @defgroup USART2_defines
  * @{
  */
typedef enum
{
	USART_Send_Ok = (uint8_t)0x00, 											/*!< USART sending successful */
	USART_Send_Failed = (uint8_t)0x01, 										/*!< USART sending failed */
	USART_NewMessage = (uint8_t)0x00, 										/*!< USART Message was received */
	USART_NoNewMessage = (uint8_t)0x01 										/*!< no new USART message received */
} USART2Status_TypeDef;

/**
  * @}
  */

/** @defgroup USART_Numbers_defines
  * @{
  */
typedef enum
{
	user_USART2 = (uint8_t)0x00,
	user_USART3 = (uint8_t)0x01
} USART_Numbers_TypeDef;

/**
  * @}
  */

/** @defgroup USART_Receive_Mode_defines
  * @{
  */
typedef enum
{
	usart_receive_normal_mode = (uint8_t)0x00,
	usart_receive_irq_mode = (uint8_t)0x01
} USART_Receive_Mode_TypeDef;

/**
  * @}
  */

typedef struct
{
	void (*uRCC_BUS_PeriphClockCmd) (uint32_t, FunctionalState); 			/* function pointer to Bank for clock */
	void (*uRCC_GPIO_Pin_RX_PeriphClockCmd) (uint32_t, FunctionalState); 	/* function pointer to Bank for RX */
	void (*uRCC_GPIO_Pin_TX_PeriphClockCmd) (uint32_t, FunctionalState); 	/* function pointer to Bank for TX */
	void (*uRCC_GPIO_Pin_CTS_PeriphClockCmd) (uint32_t, FunctionalState); 	/* function pointer to Bank for CTS */
	uint32_t uRCC_Periph_USART_BUS; 										/* assign bank to clock */
	uint32_t uRCC_Periph_GPIO_Pin_RX; 										/* assign bank to RX */
	uint32_t uRCC_Periph_GPIO_Pin_TX; 										/* assign bank to TX */
	uint32_t uRCC_Periph_GPIO_Pin_CTS; 										/* assign bank to CTS */
	uint16_t uGPIO_Pin_RX; 													/* GPIO-Pin for RX */
	uint16_t uGPIO_Pin_TX; 													/* GPIO-Pin for TX */
	uint16_t uGPIO_Pin_CTS; 												/* GPIO-Pin for CTS */
	GPIO_TypeDef* uGPIO_Bank_RX_Type; 										/* pointer to GPIO-Bank Type for RX */
	GPIO_TypeDef* uGPIO_Bank_TX_Type; 										/* pointer to GPIO-Bank Type for TX */
	GPIO_TypeDef* uGPIO_Bank_CTS_Type; 										/* pointer to GPIO-Bank Type for CTS */
	uint8_t uGPIO_Pin_RX_PinSource; 										/* GPIO-Pin Resource for RX */
	uint8_t uGPIO_Pin_TX_PinSource; 										/* GPIO-Pin Resource for TX */
	uint8_t uGPIO_Pin_CTS_PinSource; 										/* GPIO-Pin Resource for CTS */
	uint8_t uGPIO_AF_USART; 												/* assign alternate function USART */
	USART_TypeDef* uUSART_Type; 											/* pointer to USART Bezeichnung */
	USART_InitTypeDef* uglobal_USART_Struct; 								/* pointer to global struct to hold the settings */
	volatile FlagStatus* uUSART_CTS_FLAG; 									/* pointer to global flag for CTS */
	uint8_t uUSART_IRQn; 													/* USART interrupt number */
} USART_user_init_TypeDef;

/******************** functions ********************/

/* @brief  Puts character to USART port
* @param  *USARTx: Pointer to USARTx peripheral you will use
* @param  c: character to be send over USART
* @retval None
*/

/* functions to init and control USART */
void USART_init(uint8_t USARTn);
void USART_deinit(uint8_t USARTn);
void USART_CTS_enable(uint8_t USARTn);
void USART_CTS_disable(uint8_t USARTn);
void USART_set_baudrate(uint8_t USARTn, uint32_t baudrate);
uint8_t USART_Wait_Send(uint8_t USARTn);
void USART_Send(uint8_t USARTn, uint8_t Data);
void USART_Sends(uint8_t USARTn, char* Data);
uint8_t USART_Receive(uint8_t USARTn, uint8_t* Data);
uint8_t USART_IRQ_Receive(uint8_t USARTn, uint8_t* Data);

/* Funktion zur Festlegung des Interrupts der USART2 */
void USART_global_IRQ_enable(uint8_t USARTn, uint8_t Priority, uint8_t SubPriority);
void USART_global_IRQ_disable(uint8_t USARTn);
void USART_IRQ_enable(uint8_t USARTn, uint16_t IT_Flag);
void USART_IRQ_disable(uint8_t USARTn, uint16_t IT_Flag);

#endif /* __USER_USART_H */
