/*
  ******************************************************************************
  * @file      user_automat.c
  * @author    Paul Oelsner, René Rathfelder
  * @version   V1.4
  * @date      15.02.2016
  * @brief     here are all the functions for the state machine
  * 		   every state is represented through a function, and they give back a pointer to the next state function
  *
  ******************************************************************************
  */

#include "user_automat.h"
#include "user_io.h"

/******************** private state functions ********************/
static zustand_ptr Z_start(void);
static zustand_ptr Z_second(void);

/******************** global variables ********************/
volatile static automat_ptr Automat = Z_start; 					/* Pointer for functions with returnvalue, set initial value */

/******************** functions ********************/
void Automat_execute(void)										/* these function executes the actual state */
{
	Automat = (automat_ptr)Automat(); 							/* execute function and overwrite function pointer with returned pointer */
																/* cast because return type of the function pointer is automat_ptr */
	if(Automat == 0) 											/* if NULLPOINTER is set anywhere */
	{
		Automat_reset(); 										/* reset state machine */
	}
}

void Automat_reset(void)										/* these function resets the state machine completely */
{
	Automat = Z_start; 											/* execute function and overwrite functionpointer with returned functionpointer */
}

/******************** Private-Funktionen ********************/

static zustand_ptr Z_start(void)								/* starting state */
{
	automat_ptr z_ptr = Automat; 								/* set old state */

	/* FUNCTIONS FOR FIRST STATE*/

	z_ptr = Z_second;
	return (zustand_ptr)z_ptr;									/* return functionspointer */
}																/* cast because return type of the function pointer has to be void */

static zustand_ptr Z_second(void)								/* another state */
{
	automat_ptr z_ptr = Automat; 								/* set old state */

	/* FUNCTIONS FOR SECOND STATE */

	z_ptr = Z_start;
	return (zustand_ptr)z_ptr;									/* return functionpointer */
}																/* cast because return type of the function pointer has to be void */
