/*
  ******************************************************************************
  * @file      user_usart.c
  * @author    Paul Oelsner, René Rathfelder
  * @version   V1.9
  * @date      15.10.2014
  * @brief	   functions and prototypes for init and control USART on STM32F407
  ******************************************************************************
  */

#include "user_usart.h"

/******************** Globale Variablen ********************/
volatile static FlagStatus global_USART2_CTS_Flag = RESET; 						/* initialisiere CTS-Flag auf low -> senden erlaubt */
volatile static FlagStatus global_USART3_CTS_Flag = RESET; 						/* initialisiere CTS-Flag auf low -> senden erlaubt */
static USART_InitTypeDef global_USART2_Struct; 									/* leeres Struct zur Datenhaltung der USART2 erstellen */
static USART_InitTypeDef global_USART3_Struct; 									/* leeres Struct zur Datenhaltung der USART3 erstellen */

static const USART_user_init_TypeDef gUSART_Const[2] = {
													{
													RCC_APB1PeriphClockCmd, 	/* Funktionspointer auf Bank für Takt */
													RCC_AHB1PeriphClockCmd, 	/* Funktionspointer auf Bank für RX */
													RCC_AHB1PeriphClockCmd, 	/* Funktionspointer auf Bank für TX */
													RCC_AHB1PeriphClockCmd, 	/* Funktionspointer auf Bank für CTS */
													RCC_APB1Periph_USART2, 		/* Zuweisung der Bank für Takt */
													RCC_AHB1Periph_GPIOA, 		/* Zuweisung der Bank für RX */
													RCC_AHB1Periph_GPIOA, 		/* Zuweisung der Bank für TX */
													RCC_AHB1Periph_GPIOD, 		/* Zuweisung der Bank für CTS */
													GPIO_Pin_3, 				/* GPIO-Pin für RX */
													GPIO_Pin_2, 				/* GPIO-Pin für TX */
													GPIO_Pin_3, 				/* GPIO-Pin für CTS */
													GPIOA, 						/* Pointer auf GPIO-Bank Type für RX */
													GPIOA, 						/* Pointer auf GPIO-Bank Type für TX */
													GPIOD, 						/* Pointer auf GPIO-Bank Type für CTS */
													GPIO_PinSource3, 			/* GPIO-Pin Resource für RX */
													GPIO_PinSource2, 			/* GPIO-Pin Resource für TX */
													GPIO_PinSource3, 			/* GPIO-Pin Resource für CTS */
													GPIO_AF_USART2, 			/* Zuweisung für Alternative-Funktion der USART */
													USART2,						/* Pointer auf USART Bezeichnung */
													&global_USART2_Struct, 		/* Pointer auf globalen Struct zum halten der Daten */
													&global_USART2_CTS_Flag, 	/* Pointer auf globales Flag für CTS */
													USART2_IRQn 				/* USART Interrupt Nummer */
													},
													{
													RCC_APB1PeriphClockCmd, 	/* Funktionspointer auf Bank für Takt */
													RCC_AHB1PeriphClockCmd, 	/* Funktionspointer auf Bank für RX */
													RCC_AHB1PeriphClockCmd, 	/* Funktionspointer auf Bank für TX */
													RCC_AHB1PeriphClockCmd, 	/* Funktionspointer auf Bank für CTS */
													RCC_APB1Periph_USART3, 		/* Zuweisung der Bank für Takt */
													RCC_AHB1Periph_GPIOD, 		/* Zuweisung der Bank für RX */
													RCC_AHB1Periph_GPIOD, 		/* Zuweisung der Bank für TX */
													RCC_AHB1Periph_GPIOD, 		/* Zuweisung der Bank für CTS */
													GPIO_Pin_9, 				/* GPIO-Pin für RX */
													GPIO_Pin_8, 				/* GPIO-Pin für TX */
													GPIO_Pin_11, 				/* GPIO-Pin für CTS */
													GPIOD, 						/* Pointer auf GPIO-Bank Type für RX */
													GPIOD, 						/* Pointer auf GPIO-Bank Type für TX */
													GPIOD, 						/* Pointer auf GPIO-Bank Type für CTS */
													GPIO_PinSource9, 			/* GPIO-Pin Resource für RX */
													GPIO_PinSource8, 			/* GPIO-Pin Resource für TX */
													GPIO_PinSource11, 			/* GPIO-Pin Resource für CTS */
													GPIO_AF_USART3, 			/* Zuweisung für Alternative-Funktion der USART */
													USART3, 					/* Pointer auf USART Bezeichnung */
													&global_USART3_Struct, 		/* Pointer auf globalen Struct zum halten der Daten */
													&global_USART3_CTS_Flag, 	/* Pointer auf globales Flag für CTS */
													USART3_IRQn 				/* USART Interrupt Nummer */
													}
};

/******************** WEAK functions for IRQ-Handling ********************/
void USART2_RXNE_UserIRQ_Handler(void) __attribute__((weak));
void USART3_RXNE_UserIRQ_Handler(void) __attribute__((weak));

/* overwrite weak functions if not existent */
#pragma weak USART2_RXNE_UserIRQ_Handler = USART_Default_UserIRQ_Handler
#pragma weak USART3_RXNE_UserIRQ_Handler = USART_Default_UserIRQ_Handler

/* function to overwrite a nonexistent handler */
static void USART_Default_UserIRQ_Handler(void)
{
	/* do nothing */
}

/******************** functions ********************/

/* Funktion zur Initalisierung der USART des STM32F4-Discovery mit TX- und RX-Signalen
 * unter Verwendung eines Structs des Typs USART_InitTypeDef.
 * Das Struct enthält anschließend die initialiserten USART Einstellungen. */
void USART_init(uint8_t USARTn)
{
	/************************ ACTIVATE USART AFTER RESET OVER BUS ************************/
	/************ old ************/
	/* RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 				*/	/* activate clock of USART2 at APB1-Bus */
	/* RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 				*/	/* activate clock of GPIOA-Bank at AHB1-Bus */

	/************ new ************/
	gUSART_Const[USARTn].uRCC_BUS_PeriphClockCmd(gUSART_Const[USARTn].uRCC_Periph_USART_BUS, ENABLE); 			/* activate clock of USART2 at APB1-Bus */
	gUSART_Const[USARTn].uRCC_GPIO_Pin_RX_PeriphClockCmd(gUSART_Const[USARTn].uRCC_Periph_GPIO_Pin_RX, ENABLE); /* activate clock of GPIO-PIN at APB1-Bus */
	gUSART_Const[USARTn].uRCC_GPIO_Pin_TX_PeriphClockCmd(gUSART_Const[USARTn].uRCC_Periph_GPIO_Pin_TX, ENABLE); /* activate clock of GPIO-PIN at APB1-Bus */

	/************************ SETTINGS OF GPIO-PINS ************************/
	GPIO_InitTypeDef GPIO_Struct; 												/* Struct for Init GPOA-Bank */
	GPIO_Struct.GPIO_Mode = GPIO_Mode_AF; 										/* alternative mode for internal connection */
	GPIO_Struct.GPIO_OType = GPIO_OType_PP; 									/* Push-Pull output */
	GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL; 									/* no Pull-Up or Pull-Down */
	GPIO_Struct.GPIO_Speed = GPIO_Speed_25MHz; 									/* Medium-Speed */

	/************ old ************/
	/*GPIO_Struct.GPIO_Pin = GPIO_Pin_3; 									*/	/* choose Pin 3 */
	/*GPIO_Init(GPIOA, &GPIO_Struct); 										*/	/* init GPIO Pin A3 with given settings */
	/*GPIO_Struct.GPIO_Pin = GPIO_Pin_2; 									*/	/* choose Pin 2 */
	/*GPIO_Init(GPIOA, &GPIO_Struct); 										*/	/* init GPIO Pin A2 with given settings */

	/************ new ************/
	GPIO_Struct.GPIO_Pin = gUSART_Const[USARTn].uGPIO_Pin_RX;
	GPIO_Init(gUSART_Const[USARTn].uGPIO_Bank_RX_Type, &GPIO_Struct);
	GPIO_Struct.GPIO_Pin = gUSART_Const[USARTn].uGPIO_Pin_TX;
	GPIO_Init(gUSART_Const[USARTn].uGPIO_Bank_TX_Type, &GPIO_Struct);

	/************************ CONNECT GPIO-PINS WITH UART ************************/
	/************ old ************/
	/* GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); 			*/	/* connect Pin RX with USART2 */
	/* GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); 			*/	/* connect Pin TX with USART2 */

	/************ new ************/
	GPIO_PinAFConfig(gUSART_Const[USARTn].uGPIO_Bank_RX_Type, gUSART_Const[USARTn].uGPIO_Pin_RX_PinSource, gUSART_Const[USARTn].uGPIO_AF_USART);
	GPIO_PinAFConfig(gUSART_Const[USARTn].uGPIO_Bank_TX_Type, gUSART_Const[USARTn].uGPIO_Pin_TX_PinSource, gUSART_Const[USARTn].uGPIO_AF_USART);

	/************************ SETTINGS OF UART STRUCT ************************/
	/************ old ************/
	/*global_USART2_Struct.USART_BaudRate = 9600; 														*/	/* set Baudrate (Bitrate) */
	/*global_USART2_Struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 					*/	/* no flow control */
	/*global_USART2_Struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 									*/	/* use RX and TX */
	/*global_USART2_Struct.USART_Parity = USART_Parity_No; 												*/	/* no parity bit */
	/*global_USART2_Struct.USART_StopBits = USART_StopBits_1; 											*/	/* 1 stop bit */
	/*global_USART2_Struct.USART_WordLength = USART_WordLength_8b; 										*/	/* 8 Bit word length */
	/*USART_Init(USART2, &global_USART2_Struct); 														*/	/* init USART2 with given settings (give adress of struct) */
	/*USART_Cmd(USART2, ENABLE); 																		*/	/* activate USART2 */

	/************ new ************/
	gUSART_Const[USARTn].uglobal_USART_Struct->USART_BaudRate = 9600; 										/* set Baudrate (Bitrate) */
	gUSART_Const[USARTn].uglobal_USART_Struct->USART_HardwareFlowControl = USART_HardwareFlowControl_None; 	/* no flow control */
	gUSART_Const[USARTn].uglobal_USART_Struct->USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 					/* use RX and TX */
	gUSART_Const[USARTn].uglobal_USART_Struct->USART_Parity = USART_Parity_No; 								/* no parity bit */
	gUSART_Const[USARTn].uglobal_USART_Struct->USART_StopBits = USART_StopBits_1; 							/* 1 stop bit */
	gUSART_Const[USARTn].uglobal_USART_Struct->USART_WordLength = USART_WordLength_8b; 						// 8 Bit word length */
	USART_Init(gUSART_Const[USARTn].uUSART_Type, gUSART_Const[USARTn].uglobal_USART_Struct);				/* init USART with given settings (give adress of struct) */
	USART_Cmd(gUSART_Const[USARTn].uUSART_Type, ENABLE);													/* activate USART */
}

/* Funktion zur Deinitalisierung und Deaktivierung der USART2 des STM32F4-Discovery.
 * Das Struct enthält anschließend die veränderten USART2 Einstellungen. */
void USART_deinit(uint8_t USARTn)
{
	/************************ SETTINGS OF GPIO-PINS ************************/
	GPIO_InitTypeDef GPIO_Struct; 												/* Struct zum zurücksetzen des Structs */
	GPIO_StructInit(&GPIO_Struct); 												/* struct Standardwerte setzen */

	/************ old ************/
	/*GPIO_Struct.GPIO_Pin = GPIO_Pin_3; 									*/	/* Pin 3 auswählen */
	/*GPIO_Init(GPIOA, &GPIO_Struct); 										*/	/* GPIO Pin A3 mit den Einstellungen initialisieren */
	/*GPIO_Struct.GPIO_Pin = GPIO_Pin_2; 									*/	/* Pin 2 auswählen */
	/*GPIO_Init(GPIOA, &GPIO_Struct); 										*/	/* GPIO Pin A2 mit den Einstellungen initialisieren */

	/************ new ************/
	GPIO_Struct.GPIO_Pin = gUSART_Const[USARTn].uGPIO_Pin_RX;
	GPIO_Init(gUSART_Const[USARTn].uGPIO_Bank_RX_Type, &GPIO_Struct);
	GPIO_Struct.GPIO_Pin = gUSART_Const[USARTn].uGPIO_Pin_TX;
	GPIO_Init(gUSART_Const[USARTn].uGPIO_Bank_TX_Type, &GPIO_Struct);

	/************************ SETTINGS OF UART STRUCT ************************/
	/************ old ************/
	/*USART_StructInit(&global_USART2_Struct); 								*/	/* struct Standardwerte setzen  (Adresse von USART2_Struct weitergeben) */
	/*USART_DeInit(USART2); 												*/	/* USART2 deinitalisieren */
	/*USART_Cmd(USART2, DISABLE); 											*/	/* USART2 deaktivieren */

	/************ new ************/
	USART_StructInit(gUSART_Const[USARTn].uglobal_USART_Struct);
	USART_DeInit(gUSART_Const[USARTn].uUSART_Type);
	USART_Cmd(gUSART_Const[USARTn].uUSART_Type, DISABLE); 						/* USART deaktivieren */
}

/* Funktion zum aktivieren des CTS-Signals der USART2 unter Verwendung des bestehenden
 * Structs des Typs USART_InitTypeDef der init_USART2-Funktion. Pin D3 wird mit CTS der USART2 verbunden.
 * Das Struct enthält anschließend die veränderten USART2 Einstellungen. */
void USART_CTS_enable(uint8_t USARTn)
{
	//GPIOD nach Reset über BUS aktivieren
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 					//Takt der GPIOD-Bank am AHB1-Bus aktivieren
	gUSART_Const[USARTn].uRCC_GPIO_Pin_CTS_PeriphClockCmd(gUSART_Const[USARTn].uRCC_Periph_GPIO_Pin_CTS, ENABLE);

	/************************ SETTINGS OF GPIO-PIN ************************/
	GPIO_InitTypeDef GPIO_Struct; 												//Struct zur Initalisierung der GPOD-Bank
	GPIO_Struct.GPIO_Mode = GPIO_Mode_AF; 										//Alternativmodus zum internen Verbinden
	GPIO_Struct.GPIO_OType = GPIO_OType_PP; 									//Ausgang als Push-Pull festlegen
	GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL; 									//Keinen Pull-Up oder Pull-Down aktivieren
	GPIO_Struct.GPIO_Speed = GPIO_Speed_25MHz; 									//Auf Medium-Speed setzen

	/************ old ************/
	//GPIO_Struct.GPIO_Pin = GPIO_Pin_3; 										//Pin 3 auswählen
	//GPIO_Init(GPIOD, &GPIO_Struct); 											//GPIO Pin D3 mit den Einstellungen initialisieren
	//GPIO_PinAFConfig(GPIOD, GPIO_PinSource3, GPIO_AF_USART2); 				//Pin CTS mit USART2 verbinden

	/************ new ************/
	GPIO_Struct.GPIO_Pin = gUSART_Const[USARTn].uGPIO_Pin_CTS;
	GPIO_Init(gUSART_Const[USARTn].uGPIO_Bank_CTS_Type, &GPIO_Struct);
	GPIO_PinAFConfig(gUSART_Const[USARTn].uGPIO_Bank_CTS_Type, gUSART_Const[USARTn].uGPIO_Pin_CTS_PinSource, gUSART_Const[USARTn].uGPIO_AF_USART);

	/************************ CHANGE SETTINGS OF UART ************************/
	/************ old ************/
	//global_USART2_Struct.USART_HardwareFlowControl = USART_HardwareFlowControl_CTS; 	//Flusskontrolle nur über CTS
	//USART_Init(USART2, &global_USART2_Struct); 										//USART2 mit den Einstellungen initalisieren
	//USART_ClearFlag(USART2, USART_FLAG_CTS); 											//zurücksetzen des CTS-Flags

	/************ new ************/
	gUSART_Const[USARTn].uglobal_USART_Struct->USART_HardwareFlowControl = USART_HardwareFlowControl_CTS;
	USART_Init(gUSART_Const[USARTn].uUSART_Type, gUSART_Const[USARTn].uglobal_USART_Struct);
	USART_ClearFlag(gUSART_Const[USARTn].uUSART_Type, USART_FLAG_CTS);
}

/* Funktion zum deaktivieren des CTS-Signals der USART2 unter Verwendung des bestehenden
 * Structs des Typs USART_InitTypeDef der init_USART2-Funktion.
 * Das Struct enthält anschließend die veränderten USART2 Einstellungen. */
void USART_CTS_disable(uint8_t USARTn)
{
	GPIO_InitTypeDef GPIO_Struct; 														//Struct zum zurücksetzen des Structs
	GPIO_StructInit(&GPIO_Struct); 														//struct Standardwerte setzen
	//GPIO_Struct.GPIO_Pin = GPIO_Pin_3; 												//Pin 3 auswählen
	//GPIO_Init(GPIOD, &GPIO_Struct); 													//GPIO Pin D3 mit den Einstellungen initialisieren
	GPIO_Struct.GPIO_Pin = gUSART_Const[USARTn].uGPIO_Pin_CTS;
	GPIO_Init(gUSART_Const[USARTn].uGPIO_Bank_CTS_Type, &GPIO_Struct);

	//Eigenschaften der USART2 ändern
	//global_USART2_Struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 	//keine Flusskontrolle
	//USART_Init(USART2, &global_USART2_Struct); 										//USART2 mit den Einstellungen initalisieren
	gUSART_Const[USARTn].uglobal_USART_Struct->USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(gUSART_Const[USARTn].uUSART_Type, gUSART_Const[USARTn].uglobal_USART_Struct);
}

/* Funktion zum Ändern der Baudrate der USART2 unter Verwendung des bestehenden
 * Structs des Typs USART_InitTypeDef der init_USART2-Funktion.
 * Das Struct enthält anschließend die veränderten USART2 Einstellungen. */
void USART_set_baudrate(uint8_t USARTn, uint32_t baudrate)
{
	//Eigenschaften der USART2 ändern
	//global_USART2_Struct.USART_BaudRate = baudrate; 									// Baudrate (Bitrate) festlegen
	gUSART_Const[USARTn].uglobal_USART_Struct->USART_BaudRate = baudrate;
	USART_Init(USART2, &global_USART2_Struct); 											// USART mit den Einstellungen initalisieren
	USART_Init(gUSART_Const[USARTn].uUSART_Type, gUSART_Const[USARTn].uglobal_USART_Struct);
}

/* Funktion zum Warten bis Daten gesendet und nicht durch CTS blockiert. */
uint8_t USART_Wait_Send(uint8_t USARTn)
{
	uint8_t status = USART_Send_Ok; 													//setze positiven status
	set_tick_counter(2000); 															//setze counter auf 2 Sekunden

	while(USART_GetFlagStatus(gUSART_Const[USARTn].uUSART_Type, USART_FLAG_TC) == RESET || *gUSART_Const[USARTn].uUSART_CTS_FLAG == SET)
	{
		// Solange Bit nicht gesetzt (Daten nicht alle gesendet) oder blockiert durch CTS
		//...idle...
		if(get_tick_counter() == 0) 													//wenn counter abgelaufen
		{
			status = USART_Send_Failed; 												//setze negativen status
			break; 																		//breche unendliches warten ab...
		}
	}

	if(status == USART_Send_Ok)
	{
		kill_tick_counter(); 															//setze counter zurück, um keine weiteren Tick-Iterrupts zu erzeugen
	}

	return status;
}

/* Funktion zum senden von 8 Bit Daten über die USART2. */
void USART_Send(uint8_t USARTn, uint8_t Data)
{
	USART_SendData(gUSART_Const[USARTn].uUSART_Type, Data); 							//senden der Daten über die USART
	USART_Wait_Send(USARTn);
}

/* function to send a string over USARTn*/
void USART_Sends(uint8_t USARTn, char* Data)
{
	USART_Wait_Send(USARTn);
	for(int i= 0; Data[i] != 0x00; i++) {
		USART_Send(USARTn, Data[i]);
	}
}

/* Funktion zum Empfangen von 8 Bit Daten über die USART. Rückgabetyp spezifiziert neue Nachricht, Daten via Call-By-Reference.*/
uint8_t USART_Receive(uint8_t USARTn, uint8_t* Data)
{
	uint8_t status = USART_NoNewMessage; 												//setze return status

	if(USART_GetFlagStatus(gUSART_Const[USARTn].uUSART_Type, USART_FLAG_RXNE) == SET)  	//Prüfe Receive-Flag (Polling)
	{
		*Data = USART_ReceiveData(gUSART_Const[USARTn].uUSART_Type); 					//empfangen der Daten über die USART2, FLAG-RXNE wird automatisch zurückgesetzt
		//USART_ClearFlag(USART2, USART_FLAG_RXNE); 									//nicht notwendig, da durch Lesevorgang gecleared
		status = USART_NewMessage;
	}
	else 																				//wenn Flag nicht gesetzt
	{
		*Data = 0x00; 																	//überschreibe Daten mit NULL
	}

	return status;
}


/* Funktion zum Empfangen von 8 Bit Daten über die USART via Interrupt. Rückgabetyp spezifiziert neue Nachricht, Daten via Call-By-Reference.*/
uint8_t USART_IRQ_Receive(uint8_t USARTn, uint8_t* Data)
{
	*Data = USART_ReceiveData(gUSART_Const[USARTn].uUSART_Type); 						//lesen der Daten, da über IRQ-Handler bereits gültig
	return USART_NewMessage; 															//setze neue Nachricht
}


/* Funktion zum aktivieren des globalen IRQ der USART2 mit Priorität. */
void USART_global_IRQ_enable(uint8_t USARTn, uint8_t Priority, uint8_t SubPriority)
{
	//Eigenschaften des Interrupts festlegen
	NVIC_InitTypeDef USART2_IRQ_Struct; 												//Struct zur Festlegung des IRQ der USART2
	USART2_IRQ_Struct.NVIC_IRQChannel = gUSART_Const[USARTn].uUSART_IRQn; 				//festlegen des IRQ-channels für UART2 -> siehe stm32f4xx.h
	USART2_IRQ_Struct.NVIC_IRQChannelPreemptionPriority = Priority; 					//legt USART-Priorität fest
	USART2_IRQ_Struct.NVIC_IRQChannelSubPriority = SubPriority; 						//legt die USART2-Priotität fest
	USART2_IRQ_Struct.NVIC_IRQChannelCmd = ENABLE; 										//aktiviert global den den IRQ-Channel
	NVIC_Init(&USART2_IRQ_Struct); 														//IRQ mit den Einstellungen initalisieren
}

/* Funktion zum deaktivieren des globalen IRQ der USART2. */
void USART_global_IRQ_disable(uint8_t USARTn)
{
	NVIC_DisableIRQ(gUSART_Const[USARTn].uUSART_IRQn); 									//sperre USART2 Interrupts im IRQ-Controller
}

/* Funktion zum aktivieren des IRQ-Flags der USART2. */
void USART_IRQ_enable(uint8_t USARTn, uint16_t IT_Flag)
{
	USART_ITConfig(gUSART_Const[USARTn].uUSART_Type, IT_Flag, ENABLE); 					//aktivieren des Interrupts bei Änderungen des Flags für USART
}

/* Funktion zum deaktivieren des IRQ-Flags der USART2. */
void USART_IRQ_disable(uint8_t USARTn, uint16_t IT_Flag)
{
	USART_ITConfig(gUSART_Const[USARTn].uUSART_Type, IT_Flag, DISABLE); 				//deaktivieren des Interrupts bei Änderungen des Flags für USART
}

/******************** IRQ-Handler Funktionen ********************/

/* Funktion welche von der Interrupt-Service-Routine der USART2 aufgerufen wird.
 * Abfrage und ggf. zurücksetzen der Interrup-Flags. */
void USART2_IRQHandler(void) 															//Für Bezeichnung siehe  startup_stm32f4xx.c
{
	NVIC_DisableIRQ(USART2_IRQn); 														//sperre USART2 Interrupts im IRQ-Controller

	if(USART_GetITStatus(USART2, user_USART_CTS) == SET)   								//wenn CTS-Flanke steigt oder fällt
	{
		USART_ClearITPendingBit(USART2, user_USART_CTS); 								//zurücksetzen des Flags

		if(global_USART2_CTS_Flag == RESET)   											//wenn low
		{
			global_USART2_CTS_Flag = SET; 												//setze high
		}
		else //wenn high
		{
			global_USART2_CTS_Flag = RESET; 											//setze low
		}
	}

	if(USART_GetITStatus(USART2, user_USART_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART2, user_USART_RXNE); 								//zurücksetzen des Flags
		//erinnerung RXNE-Flag wird beim auslesen des empfangsregisters automatisch zurückgesetzt
		USART2_RXNE_UserIRQ_Handler(); 													//eigenen Handler ausführen
	}

	NVIC_EnableIRQ(USART2_IRQn);  														//freigeben der USART2-Interrupts im IRQ-Controller
}

/* Funktion welche von der Interrupt-Service-Routine der USART3 aufgerufen wird.
 * Abfrage und ggf. zurücksetzen der Interrup-Flags. */
void USART3_IRQHandler(void) 															//Für Bezeichnung siehe  startup_stm32f4xx.c
{
	NVIC_DisableIRQ(USART3_IRQn); 														//sperre USART2 Interrupts im IRQ-Controller

	if(USART_GetITStatus(USART3, user_USART_CTS) == SET)   								//wenn CTS-Flanke steigt oder fällt
	{
		USART_ClearITPendingBit(USART3, user_USART_CTS); 								//zurücksetzen des Flags

		if(global_USART3_CTS_Flag == RESET)   											//wenn low
		{
			global_USART3_CTS_Flag = SET; 												//setze high
		}
		else 																			//wenn high
		{
			global_USART3_CTS_Flag = RESET; 											//setze low
		}
	}

	if(USART_GetITStatus(USART3, user_USART_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART3, user_USART_RXNE); 								//zurücksetzen des Flags
		//erinnerung RXNE-Flag wird beim auslesen des empfangsregisters automatisch zurückgesetzt
		USART3_RXNE_UserIRQ_Handler(); 													//eigenen Handler ausführen
	}

	NVIC_EnableIRQ(USART3_IRQn);  														//freigeben der USART2-Interrupts im IRQ-Controller
}
