/*
  ******************************************************************************
  * @file      main.c
  * @author    René Rathfelder
  * @version   V1.0
  * @date      15.02.2016
  * @brief     main function
  * 		   Hint: "cmsis_boot/stm32f4xx.h", "cmsis_boot/system_stm32f4xx.c" have to be set at 8MHz!
  ******************************************************************************
  */

#include "main.h"

/******************** private functions ********************/
/* static void set_CAN1_Filter_with_Database_CANIDs(void); */
/******************** main function ********************/

int main(void)
{
	/* ----- System -----*/
	SystemInit(); 											/* set external 8MHz Crystal */
	SystemCoreClockUpdate(); 								/* update SystemCoreClock after changing systemclock */
	NVIC_SetPriorityGrouping(4); 							/* set interrupt groups, 3 Bit for priority and 1 Bit for SubPriority */

	/* ----- USART2 ----- */
	USART_init(user_USART2);								/* init USART2 */
	USART_set_baudrate(user_USART2, 57600); 				/* set Baudrate */
	/*USART_CTS_enable(user_USART2); 					*/	/* activate CTS-Signal */
	/*USART_global_IRQ_enable(user_USART2, 0, 0); 		*/	/* highest priority for USART2 IRQ */
	/*USART_IRQ_enable(user_USART2, user_USART_CTS); 	*/	/* IRQ für CTS-Signal ermöglichen */
	/*USART_IRQ_enable(user_USART2, user_USART_RXNE); 	*/	/* IRQ für RXNE ermöglichen */

	/* ----- USART3 ----- */
	/*USART_init(user_USART3); 							*/	/* USART2 initalisieren */
	/*USART_set_baudrate(user_USART3, 57600); 			*/	/* Baudrate festlegen */
	/*USART_CTS_enable(user_USART3); 					*/	/* CTS-Signal aktivieren */
	/*USART_global_IRQ_enable(user_USART3, 0, 0); 		*/	/* IRQ für USART2 mit höchster Priority und höchster SubPriority freigeben */
	/*USART_IRQ_enable(user_USART3, user_USART_CTS); 	*/	/* IRQ für CTS-Signal ermöglichen */

	/* ---- IO ---- */
	io_init();

	/* ----- INIT STATI ------ */
	/* uint8_t init_status_can1 = 0; 					*/	/* set status for CAN1 Init */

	/*while(init_status_can1 == 0)						*/	/* loop till CAN1 is initialized correctly */
	/*{													*/
		/* ----- set CAN1-Filter and init CAN1 ----- 	*/
		/*if(CAN1_init() == CAN1_init_succ) 			*/	/* CAN1 initialisieren */
		/*{												*/
		/*	init_status_can1 = 1; 						*/	/* set Success status flag */
		/*	set_CAN1_Filter_with_Database_CANIDs();		*/

		/*	CAN1_set_baudrate(baud_1000); 				*/	/* set baudrate to 1000 kbps */
		/*	CAN1_global_RX0_IRQ_enable(4, 0); 			*/	/* IRQ for CAN1 Priority SubPriority */
		/*	CAN1_global_RX1_IRQ_enable(4, 0); 			*/	/* IRQ for CAN1 Priority SubPriority */
		/*	CAN1_IRQ_enable(CAN1_IRQ_FMP0); 			*/	/* CAN1-IRQ if Nachricht in Fifo */
		/*	//CAN1_IRQ_enable(CAN1_IRQ_FF0); 			*/	/* CAN1-IRQ if Fifo full */
		/*	//CAN1_IRQ_enable(CAN1_IRQ_FOV0); 			*/	/* CAN1-IRQ if Fifo overflow */
		/*	CAN1_IRQ_enable(CAN1_IRQ_FMP1); 			*/	/* CAN1-IRQ if message in Fifo */
		/*	//CAN1_IRQ_enable(CAN1_IRQ_FF1); 			*/	/* CAN1-IRQ if Fifo full */
		/*	//CAN1_IRQ_enable(CAN1_IRQ_FOV1); 			*/	/* CAN1-IRQ if Fifo overflow */
		/*}												*/
		/*else											*/
		/*{												*/
		/*	init_status_can1 = 0; 						*/	/* set error status flag */
		/*}												*/
		/*if (init_status_can1 == 0)					*/	/* check CAN Status -> flash led if ok */
		/*{												*/
			/*uint8_t i = 0; 							*/	/* loopcounter */
			/*for(i = 0; i < 10; i++) 					*/	/* loop 10 times for 10 led flashes */
			/*{											*/
				/*tick_delay(125); 						*/	/* wait */
				/*io_led_toggle(led_green);				*/
			/*}											*/
		/*}												*/
	/*}													*/

	io_led_on(led_green);									/* set green LED when init complete or no button pressed */
	USART_Sends(user_USART2, "Init complete - starting state machine.\n");


	while(1)												/* start mainloop */
	{
		if(io_get_button() == 1) 							/* when button pressed */
		{
			Automat_reset();								/* reset state machine */
			if(get_tick_counter() == 0)  					/* check counter */
			{
				set_tick_counter(500); 						/* set counter */
			}
		}
		else 												/* when button not pressed */
		{

			Automat_execute();								/* start the state machine*/
			tick_delay(100); 								/* wait */
		}
	}
}

/******************** private functions ********************/
