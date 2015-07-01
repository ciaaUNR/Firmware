/* Copyright 2014, Mariano Cerdeiro
 * Copyright 2014, Pablo Ridolfi
 * Copyright 2014, Juan Cecconi
 * Copyright 2014, Gustavo Muro
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Blinking_echo example source file
 **
 ** This is a mini example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Blinking Blinking_echo example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * MaCe         Mariano Cerdeiro
 * PR           Pablo Ridolfi
 * JuCe         Juan Cecconi
 * GMuro        Gustavo Muro
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20141019 v0.0.2   JuCe add printf in each task,
 *                        remove trailing spaces
 * 20140731 v0.0.1   PR   first functional version
 */

/*==================[inclusions]=============================================*/


#include <stdio.h>
#include <stdlib.h>


#include "os.h"               /* <= operating system header */
#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
#include "ciaaPOSIX_string.h" /* <= string header */
#include "ciaak.h"            /* <= ciaa kernel header */
#include "blinking.h"         /* <= own header */

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/** \brief File descriptor for digital output ports
 *
 * Device path /dev/dio/out/0
 */
static int32_t fd_out, fd_in, fd_gpio_out, fd_gpio_in, fd_keyb;
static uint32_t Periodic_Task_Counter;
static uint8 pulsador1=0;
int (*functions[4][4])(void);

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

uint8_t ciaaKeyboard_lineChange(void);
int ciaaKeyboard_ReadCol(void);
void ciaaKeyboard_FunctionInit(void);

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
int main(void)
{
   /* Starts the operating system in the Application Mode 1 */
   /* This example has only one Application Mode */

   StartOS(AppMode1);

   /* StartOs shall never returns, but to avoid compiler warnings or errors
    * 0 is returned */
   return 0;
}

/** \brief Error Hook function
 *
 * This fucntion is called from the os if an os interface (API) returns an
 * error. Is for debugging proposes. If called this function triggers a
 * ShutdownOs which ends in a while(1).
 *
 * The values:
 *    OSErrorGetServiceId
 *    OSErrorGetParam1
 *    OSErrorGetParam2
 *    OSErrorGetParam3
 *    OSErrorGetRet
 *
 * will provide you the interface, the input parameters and the returned value.
 * For more details see the OSEK specification:
 * http://portal.osek-vdx.org/files/pdf/specs/os223.pdf
 *
 */
void ErrorHook(void)
{
   ciaaPOSIX_printf("ErrorHook was called\n");
   ciaaPOSIX_printf("Service: %d, P1: %d, P2: %d, P3: %d, RET: %d\n", OSErrorGetServiceId(), OSErrorGetParam1(), OSErrorGetParam2(), OSErrorGetParam3(), OSErrorGetRet());
   ShutdownOS(0);
}

/** \brief Initial task
 *
 * This task is started automatically in the application mode 1.
 */
TASK(InitTask)
{
   /* init CIAA kernel and devices */
   ciaak_start();

   /* print message (only on x86) */
   ciaaPOSIX_printf("Init Task...\n");

   /* open CIAA GPIO inputs-outputs */

   fd_gpio_out = ciaaPOSIX_open("/dev/gpio/out/0", O_RDWR);

   fd_gpio_in = ciaaPOSIX_open("/dev/gpio/in/0", O_RDONLY);

   /* open CIAA digital inputs-outputs */

   fd_out = ciaaPOSIX_open("/dev/dio/out/0", O_RDWR);

   fd_in = ciaaPOSIX_open("/dev/dio/in/0", O_RDONLY);

   /* open CIAA Keyboard from PONCHO  */

   fd_keyb = ciaaPOSIX_open("/dev/keyb/keyb0",O_RDONLY);
   if(fd_keyb == NULL){
      /* Error. Keyboard can not be opened */
      printf("Error creating OS Thread timer!\n");
      while(1);
   }

   /* activate periodic task:
    *  - for the first time after 350 ticks (350 ms)
    *  - and then every 250 ticks (250 ms)
    */

   Periodic_Task_Counter = 0;
   SetRelAlarm(ActivatePeriodicTask, 350, 250);

   /* terminate task */
   TerminateTask();
}

/** \brief Periodic Task
 *
 * This task is started automatically every time that the alarm
 * ActivatePeriodicTask expires.
 *
 */
TASK(PeriodicTask)
{
   uint8 outputs, inputs;
   uint16 gpoutputs;
   int32_t result;

   ciaaPOSIX_printf("Blinking\n");

   /* Secuencia Aleatoria */
   /*
   outputs = (1<<(rand()%6));
   ciaaPOSIX_write(fd_out, &outputs, 1);
   outputs = 0;
   */
   /***********************/

   /* Escribe las 9 GPIO Salidas de manera intermitente */
   /*
   ciaaPOSIX_read(fd_gpio_out, &gpoutputs, 1);
   gpoutputs ^= 0x1FF;
   ciaaPOSIX_write(fd_gpio_out, &gpoutputs, 1);
   */
   /*****************************************************/

   /* Uso ioclt para cambiar la GPIO 0 a entrada */
   result = ciaaPOSIX_ioctl(fd_gpio_out, ciaaPOSIX_IOCTL_GPIO_IN, ciaaGPIO_0);

   /* Leo las entradas GPIO */
   /* Puenteando GPIO y GND devuelve 0, sino 1 */
   ciaaPOSIX_read(fd_gpio_in, &gpoutputs, 1);

   /* Leo las salidas GPIO */
   ciaaPOSIX_read(fd_gpio_out, &gpoutputs, 1);


   /* Lectura Pulsador 1 */

   ciaaPOSIX_read(fd_in, &inputs, 1);

   if(inputs == 1){
	   if(pulsador1==0){
		   pulsador1=1;
	   }else{
		   pulsador1=0;
	   }
   }

   /* Secuencia de los 6 leds */
   /* La inicio o pauso con el pulsador 1 */

   if(pulsador1 == 1){

	   ciaaPOSIX_read(fd_out, &outputs, 1);

	   outputs ^= 1<<Periodic_Task_Counter;

	   ciaaPOSIX_write(fd_out, &outputs, 1);

	   Periodic_Task_Counter++;
	   if(Periodic_Task_Counter>5){
		   Periodic_Task_Counter=0;
	   }
   }


   /* terminate task */
   TerminateTask();
}

/* Keyboard Code */

TASK(Keyboard_Handler){
      ciaaKeyboard_MainTask();
      TerminateTask();
}




void ciaaKeyboard_MainTask(void){

    uint8_t Row = 0;
    static uint8_t prev_col = -1;
    static uint8_t prev_row = -1;

    int Column;
    //aca falta moverlo a otro lado
    ciaaKeyboard_FunctionInit();

	Row = ciaaKeyboard_lineChange();
	Column = ciaaKeyboard_ReadCol();


	if((Row != prev_row) && (Column != prev_col))
	{
	    ciaaKeyboard_Action(Row,Column);

	    prev_row = Row;
	    prev_col = Column;
	}

}



/** \brief Keyboard Active Line Change
 *
 * This function changes the active output of the keyboard
 * The keyboard works with negative logic, so a low level output
 * is taken as active. It returns the active line:
 * counter == 0  --> T_FIL0 = 0, T_FIL1 = 1, T_FIL2 = 1, T_FIL3 = 1
 * counter == 1  --> T_FIL0 = 1, T_FIL1 = 0, T_FIL2 = 1, T_FIL3 = 1
 * counter == 2  --> T_FIL0 = 1, T_FIL1 = 1, T_FIL2 = 0, T_FIL3 = 1
 * counter == 3  --> T_FIL0 = 1, T_FIL1 = 1, T_FIL2 = 1, T_FIL3 = 0
 */


uint8_t ciaaKeyboard_lineChange(void){
      static uint8 counter = 4;

      if(counter < 4) counter++;
      else counter = 0;

      switch(counter){
      case 0:
    	 ciaaPOSIX_write(fd_keyb,0xFE,1);
      	 break;
      case 1:
    	 ciaaPOSIX_write(fd_keyb,0xFD,1);
    	 break;
      case 2:
    	 ciaaPOSIX_write(fd_keyb,0xFB,1);
    	 break;
      case 3:
    	 ciaaPOSIX_write(fd_keyb,0xF7,1);
    	 break;
      }
      return counter;
}

/** \brief Read Keyboard Column
 *
 * This function returns the column value active. The read value
 * isn't the column value, so we should make a convertion
 * result == 0x0E  --> COL0 is pressed
 * result == 0x0D  --> COL1 is pressed
 * result == 0x0B  --> COL2 es pressed
 * result == 0x07  --> COL3 es pressed
 * result == -1    --> No column pressed
 */

int ciaaKeyboard_ReadCol(void){

	int result = 0;
	ciaaPOSIX_read(fd_keyb,&result,1);
	result |= 0xF0;
	result = ~result;
	switch(result){
	case 1:
		result = 0;
		break;
	case 2:
		result = 1;
		break;
	case 4:
		result = 2;
		break;
	case 8:
		result = 3;
		break;
	default:
		result = -1;
		break;
	}
	return result;
}




/** \brief Keyboard Take Action Function
 *
 * This function executes the action, given from Row,Column Function.
 * If Column is -1, then, nothing is executed.
 ** \param[in]  Keyboard Row pressed
 ** \param[in]  Keyboard Column pressed
 */
void ciaaKeyboard_Action(uint8_t Row, int Column){
   static int Prev_Column=5;
   if (Column != (-1)){
      if(Prev_Column != Column){
         functions[Row][Column]();
         Prev_Column = Column;
      }
   }
}

void ciaaKeyboard_FunctionInit(void){
    functions[0][0]=ciaaKeyboard_MainTask;
    functions[0][1]=ciaaKeyboard_MainTask;
    functions[0][2]=ciaaKeyboard_MainTask;
    functions[0][3]=ciaaKeyboard_MainTask;
    functions[1][0]=ciaaKeyboard_MainTask;
    functions[1][1]=ciaaKeyboard_MainTask;
    functions[1][2]=ciaaKeyboard_MainTask;
    functions[1][3]=ciaaKeyboard_MainTask;
    functions[2][0]=ciaaKeyboard_MainTask;
    functions[2][1]=ciaaKeyboard_MainTask;
    functions[2][2]=ciaaKeyboard_MainTask;
    functions[2][3]=ciaaKeyboard_MainTask;
    functions[3][0]=ciaaKeyboard_MainTask;
    functions[3][1]=ciaaKeyboard_MainTask;
    functions[3][2]=ciaaKeyboard_MainTask;
    functions[3][3]=ciaaKeyboard_MainTask;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
