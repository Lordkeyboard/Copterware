/*
  ******************************************************************************
  * @file      user_io.c
  * @author    Paul Oelsner, René Rathfelder
  * @version   V1.1
  * @date      15.02.2016
  * @brief     functions to get and set in- and output pins
  ******************************************************************************
*/

#include "user_io.h"

/******************** WEAK functions for IRQ-Handling ********************/
void Button_UserIRQ_Handler(void) __attribute__((weak));
#pragma weak Button_UserIRQ_Handler = Button_Default_UserIRQ_Handler	/* overwrite weak functions if nonexistent */
static void Button_Default_UserIRQ_Handler(void)						/* function to overwrite a nonexistent handler */
{
	/* do nothing */
}

/******************** global variables ********************/
volatile static uint16_t led_toggle_count[led_max_count] = {0, 0, 0, 0};

/******************** functions ********************/

void io_init(void)														/* initializing GPIOs */
{
/* ---- Outputs ---- */
	GPIO_InitTypeDef GPIO_Struct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Struct.GPIO_Pin = led_green | led_orange | led_red | led_blue; /* configure all four led pins */
	GPIO_Struct.GPIO_Mode = GPIO_Mode_OUT; 								/* output mode */
	GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz; 							/* set GPIO modules clock speed */
	GPIO_Struct.GPIO_OType = GPIO_OType_PP; 							/* set pin type to push / pull (as opposed to open drain) */
	GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL; 							/* set the pullup / pulldown resistors to be inactive */
	GPIO_Init(GPIOD, &GPIO_Struct); 									/* pass all values to GPIO_Init function */

/* ---- Inputs ---- */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_Struct.GPIO_Pin = io_button;		  							/* configure the IO Button (PA0) */
	GPIO_Struct.GPIO_Mode = GPIO_Mode_IN; 	  							/* input mode */
	GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;							/* set GPIO modules clock speed */
	GPIO_Struct.GPIO_OType = GPIO_OType_PP;   							/* set pin type to push / pull (as opposed to open drain) */
	GPIO_Struct.GPIO_PuPd = GPIO_PuPd_DOWN;   							/* enable the pulldown resistor --> we want to detect a high level */
	GPIO_Init(GPIOA, &GPIO_Struct);			  							/* pass all values to GPIO_Init function */

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0); 		/* Button(PA0) as external interrupt to sysconfig */

/* ---- extInt config ---- */
    EXTI_InitTypeDef Button_EXTIRQ_Struct; 								/* Struct for IRQ settings for the Button */
	Button_EXTIRQ_Struct.EXTI_Line = EXTI_Line0; 						/* connect Button with IRQ */
	Button_EXTIRQ_Struct.EXTI_LineCmd = ENABLE; 						/* enable external Interrupt */
	Button_EXTIRQ_Struct.EXTI_Mode = EXTI_Mode_Interrupt; 				/* Interrupt mode -> no Event mode */
	Button_EXTIRQ_Struct.EXTI_Trigger = EXTI_Trigger_Rising; 			/* activate when button pressed */
    EXTI_Init(&Button_EXTIRQ_Struct); 									/* initialize external Interrupt */

/* ---- old extInt config ---- */
	/*NVIC_InitTypeDef Button_IRQ_Struct;
	Button_IRQ_Struct.NVIC_IRQChannel = EXTI0_IRQn;
	Button_IRQ_Struct.NVIC_IRQChannelPreemptionPriority = 1;
	Button_IRQ_Struct.NVIC_IRQChannelSubPriority = 0;
	Button_IRQ_Struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&Button_IRQ_Struct);
	*/

	NVIC_SetPriority(EXTI0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0)); /* PreemptionPriority, SubPriority */
	NVIC_EnableIRQ(EXTI0_IRQn);
}


uint8_t io_get_button(void)												/* function for getting the buttonstatus */
{
	uint8_t status = io_button_set;
	if(GPIO_ReadInputDataBit(GPIOA, io_button) == RESET)
	{
		status = io_button_reset;
	}
	return status;
}

void io_led_toggle(uint16_t LED)										/* function for toggling leds */
{
	GPIO_ToggleBits(GPIOD, LED);
}

void io_led_on(uint16_t LED)											/* function for turning on leds */
{
	GPIO_SetBits(GPIOD, LED);
}

void io_led_off(uint16_t LED)											/* function for turning off leds */
{
	GPIO_ResetBits(GPIOD, LED);
}


void io_led_toggle_count_set(uint8_t LED_COUNT, uint16_t max)			/* function for setting led counter */
{
	uint16_t LED = 0;
	uint8_t status = 1;

	switch(LED_COUNT)
	{
		case led_green_count:
			LED = led_green;
			break;

		case led_orange_count:
			LED = led_orange;
			break;

		case led_red_count:
			LED = led_red;
			break;

		case led_blue_count:
			LED = led_blue;
			break;

		default:
			status = 0;
			break;
	}

	if(LED_COUNT < led_max_count && status == 1)
	{
		if(led_toggle_count[LED_COUNT] != max)
		{
			led_toggle_count[LED_COUNT] += 1;
		}
		else
		{
			led_toggle_count[LED_COUNT] = 0;
			io_led_toggle(LED);
		}
	}
}

void io_led_toggle_count_reset(uint8_t LED_COUNT)						/* function for resetting led counter*/
{
	if(LED_COUNT < led_max_count)
	{
		led_toggle_count[LED_COUNT] = 0;
	}
}

/******************** IRQ-Handler functions ********************/

/* function call from EXTI0 */
/* getting and/or resetting interrupt flag */
void EXTI0_IRQHandler(void) 											/* naming see startup_stm32f4xx.c */
{
	NVIC_DisableIRQ(EXTI0_IRQn); 										/* disable EXTI0 interrupts in the  IRQ-Controller */
    if (EXTI_GetITStatus(EXTI_Line0) == SET) 							/* if IRQ-Flag is set */
    {
        EXTI_ClearITPendingBit(EXTI_Line0); 							/* reset flag */
		Button_UserIRQ_Handler(); 										/* execute own handler */
    }
	NVIC_EnableIRQ(EXTI0_IRQn);  										/* free the EXTI0-interrupts in IRQ-Controller */
}
