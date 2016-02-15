/*
  ******************************************************************************
  * @file      user_automat.h
  * @author    Paul Oelsner
  * @version   V1.4
  * @date      09.09.2014
  * @brief	   functions and prototypes for a state machine
  ******************************************************************************
  */

#ifndef __USER_AUTOMAT_H
#define __USER_AUTOMAT_H

/******************** includes ********************/
#include "stm32f4xx.h" 									/* for uint definition */
#include "user_io.h" 									/* user lib für Board Button und LEDs */
#include "user_timer.h" 								/* functions for Hardware-Timer */

/******************** defines *********************/
typedef void (*zustand_ptr)(void); 						/* Pointer for functions withour return value */
typedef zustand_ptr(*automat_ptr)(void);  				/* Pointer for functions with return value of a pointer for functions without return value */

/******************** functions ********************/
void Automat_execute(void);
void Automat_reset(void);

#endif // __USER_AUTOMAT_H
