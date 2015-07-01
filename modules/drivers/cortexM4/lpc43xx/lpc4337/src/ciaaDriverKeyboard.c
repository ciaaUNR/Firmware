/* Copyright 2015, Juan Pablo Vecchio (UNR-FCEIA)
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

/** \brief CIAA GPIO Driver for LPC4337
 **
 ** Implements the General Purpose Input/Output (GPIO) Driver for LPC4337
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Drivers CIAA Drivers
 ** @{ */
/** \addtogroup GPIO GPIO Drivers
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * JPV           Juan Pablo Vecchio
 *JJR			Joaquin Rodriguez
 *
 */


/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20150612 v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "ciaaDriverKeyboard.h"
#include "ciaaDriverKeyboard_Internal.h"
#include "ciaaPOSIX_stdlib.h"
#include "ciaaPOSIX_string.h"
#include "chip.h"

/*==================[macros and definitions]=================================*/
/** \brief Pointer to Devices */
typedef struct  {
   ciaaDevices_deviceType * const * const devices;
   uint8_t countOfDevices;
} ciaaDriverConstType;

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
/** \brief Device for Keyboard 0 */
static ciaaDevices_deviceType ciaaDriverKeyboard_keyb0 = {
   "keyb0",                             /** <= driver name */
   ciaaDriverKeyboard_open,              /** <= open function */
   ciaaDriverKeyboard_close,             /** <= close function */
   ciaaDriverKeyboard_read,              /** <= read function */
   ciaaDriverKeyboard_write,             /** <= write function */
   ciaaDriverKeyboard_ioctl,             /** <= ioctl function */
   NULL,                                 /** <= seek function is not provided */
   NULL,                                 /** <= upper layer */
   (void*)&ciaaDriverKeyboard_keyb,     /** <= layer */
   NULL                                  /** <= NULL no lower layer */
};

static ciaaDevices_deviceType * const ciaaKeyboardDevices[] = {
   &ciaaDriverKeyboard_keyb0,
};

static ciaaDriverConstType const ciaaDriverKeyboardConst = {
   ciaaKeyboardDevices,
   1
};

/*==================[external data definition]===============================*/
/** \brief Keyboard 0 */
ciaaDriverKeyboard_keybType ciaaDriverKeyboard_keyb;


/*==================[internal functions definition]==========================*/

void ciaa_lpc4337_Keyboard_init(void)
{
   Chip_GPIO_Init(LPC_GPIO_PORT);

#if (BOARD == ciaa_nxp)
   /* GPIO */

#elif (BOARD == edu_ciaa_nxp)
   /* GPIO */

#if (PONCHO == interfaz_de_usuario)
   /* Keyboard Initialization */
     Chip_SCU_PinMux(1,5,MD_PUP|MD_EZI|MD_ZI,FUNC0);   /* GPIO1[8],  T_COL0 */
     Chip_SCU_PinMux(7,4,MD_PUP|MD_EZI|MD_ZI,FUNC0);   /* GPIO3[12], T_COL1 */
     Chip_SCU_PinMux(7,5,MD_PUP|MD_EZI|MD_ZI,FUNC0);   /* GPIO3[13], T_COL2 */
     Chip_SCU_PinMux(6,12,MD_PUP|MD_EZI|MD_ZI,FUNC0);  /* GPIO2[8],  T_COL3 */

     Chip_SCU_PinMux(4,0,MD_PUP,FUNC0);   /* GPIO2[0],  T_FIL0 */
     Chip_SCU_PinMux(4,1,MD_PUP,FUNC0);   /* GPIO2[1],  T_FIL1 */
     Chip_SCU_PinMux(4,2,MD_PUP,FUNC0);   /* GPIO2[2],  T_FIL2 */
     Chip_SCU_PinMux(4,3,MD_PUP,FUNC0);   /* GPIO2[3],  T_FIL3 */

     Chip_GPIO_SetDir(LPC_GPIO_PORT, 2,(1<<0)|(1<<1)|(1<<2)|(1<<3),1);

     Chip_GPIO_SetDir(LPC_GPIO_PORT, 1,(1<<8),0);
     Chip_GPIO_SetDir(LPC_GPIO_PORT, 3,(1<<12)|(1<<13),0);
     Chip_GPIO_SetDir(LPC_GPIO_PORT, 2,(1<<8),0);

     Chip_GPIO_ClearValue(LPC_GPIO_PORT, 2,(1<<0)|(1<<1)|(1<<2)|(1<<3));

      /* GPIO IRQ Initialization*/
      /* Matrix Keyboard has 4-pins as input (columns) and 4-pins as output (rows) */
      /* We need to generate interrupts from inputs pin */
     /**
      * @brief	GPIO Interrupt Pin Select
      * @param	PortSel	: GPIO PINTSEL interrupt, should be: 0 to 7
      * @param	PortNum	: GPIO port number interrupt, should be: 0 to 7
      * @param	PinNum	: GPIO pin number Interrupt , should be: 0 to 31
      * @return	Nothing
      * void Chip_SCU_GPIOIntPinSel(uint8_t PortSel, uint8_t PortNum, uint8_t PinNum);
      */
     Chip_SCU_GPIOIntPinSel(0,1,8);
     Chip_SCU_GPIOIntPinSel(1,3,12);
     Chip_SCU_GPIOIntPinSel(2,3,13);
     Chip_SCU_GPIOIntPinSel(3,2,8);

#endif

#else
   #error please define BOARD variable!
#endif
}

void ciaa_lpc4337_writeKeyboard(uint32_t outputNumber, uint32_t value)
{
#if (BOARD == edu_ciaa_nxp)
#if (PONCHO == interfaz_de_usuario)
	switch(outputNumber)
	   {
	      case 0: /* GPIO0 */
	         if(value)
	         {
	            Chip_GPIO_SetValue(LPC_GPIO_PORT, 2, 1<<0);
	         }
	         else
	         {
	            Chip_GPIO_ClearValue(LPC_GPIO_PORT,2, 1<<0);
	         }
	         break;
	      case 1: /* GPIO1 */
	         if(value)
	         {
	            Chip_GPIO_SetValue(LPC_GPIO_PORT, 2, 1<<1);
	         }
	         else
	         {
	            Chip_GPIO_ClearValue(LPC_GPIO_PORT, 2, 1<<1);
	         }
	         break;
	      case 2: /* GPIO2 */
	         if(value)
	         {
	            Chip_GPIO_SetValue(LPC_GPIO_PORT, 2, 1<<2);
	         }
	         else
	         {
	            Chip_GPIO_ClearValue(LPC_GPIO_PORT, 2, 1<<2);
	         }
	         break;
	      case 3: /* GPIO3 */
	         if(value)
	         {
	            Chip_GPIO_SetValue(LPC_GPIO_PORT, 2, 1<<3);
	         }
	         else
	         {
	            Chip_GPIO_ClearValue(LPC_GPIO_PORT, 2, 1<<3);
	         }
	         break;
	     }
#endif
#endif
}

/*==================[external functions definition]==========================*/
extern ciaaDevices_deviceType * ciaaDriverKeyboard_open(char const * path,
      ciaaDevices_deviceType * device, uint8_t const oflag)
{
   return device;
}

extern int32_t ciaaDriverKeyboard_close(ciaaDevices_deviceType const * const device)
{
   return 0;
}

extern int32_t ciaaDriverKeyboard_ioctl(ciaaDevices_deviceType const * const device, int32_t const request, void * param)
{
   /* Hay que hacerlo todavía. Posiblemente, el teclado use interrupciones */
	return -1;
}

extern uint16_t ciaaDriverKeyboard_read(ciaaDevices_deviceType const * const device, uint16_t * buffer, size_t size)
{
   ssize_t ret = -1;

   /* Can't store read result in buffer. At least 1 byte required. */
   if(size != 0)
   {
      if(device == ciaaKeyboardDevices[0])
      {
#if(BOARD == edu_ciaa_nxp)
#if (PONCHO == none)
         buffer[0]  = Chip_GPIO_GetPinState(LPC_GPIO_PORT, 3, 0)  ? 0 : 1;
         buffer[0] |= Chip_GPIO_GetPinState(LPC_GPIO_PORT, 3, 3)  ? 0 : 2;
         buffer[0] |= Chip_GPIO_GetPinState(LPC_GPIO_PORT, 3, 4)  ? 0 : 4;
         buffer[0] |= Chip_GPIO_GetPinState(LPC_GPIO_PORT, 5, 15) ? 0 : 8;
         buffer[0] |= Chip_GPIO_GetPinState(LPC_GPIO_PORT, 5, 16) ? 0 : 16;
         buffer[0] |= Chip_GPIO_GetPinState(LPC_GPIO_PORT, 3, 5)  ? 0 : 32;
         buffer[0] |= Chip_GPIO_GetPinState(LPC_GPIO_PORT, 3, 6)  ? 0 : 64;
         buffer[0] |= Chip_GPIO_GetPinState(LPC_GPIO_PORT, 3, 7)  ? 0 : 128;
         buffer[0] |= Chip_GPIO_GetPinState(LPC_GPIO_PORT, 2, 8)  ? 0 : 256;
#endif

#if (PONCHO == interfaz_de_usuario)
         buffer[0]  = Chip_GPIO_GetPinState(LPC_GPIO_PORT, 1, 8)  ? 0 : 1;
         buffer[0] |= Chip_GPIO_GetPinState(LPC_GPIO_PORT, 3, 12) ? 0 : 2;
         buffer[0] |= Chip_GPIO_GetPinState(LPC_GPIO_PORT, 3, 13) ? 0 : 4;
         buffer[0] |= Chip_GPIO_GetPinState(LPC_GPIO_PORT, 2, 8)  ? 0 : 8;
#endif
#endif

         /* 1 byte read */
         ret = 1;
      }
      else {
    	  /* Put your code if you have another device */
      }
   }
   return ret;
}

extern uint16_t ciaaDriverKeyboard_write(ciaaDevices_deviceType const * const device, uint16_t const * const buffer, size_t const size)
{
	/* we just can write the first 4 bits. Others are dismiss. */
   ssize_t ret = -1;

   if(size != 0)
   {
      if(device == ciaaKeyboardDevices[0])
      {
         int32_t i;

         for(i = 0; i < size*8; i++)
         {
            ciaa_lpc4337_writeKeyboard(i, buffer[0] & (1 << i));
         }

         /* save actual output state in layer data */
         *((ciaaDriverKeyboard_keybType *)device->layer) = buffer[0];

         /* 1 byte written */
         ret = 1;
      }
   }
   return ret;
}

void ciaaDriverKeyboard_init(void)
{
   uint8_t loopi;

   ciaa_lpc4337_Keyboard_init();

   /* add dio driver to the list of devices */
   for(loopi = 0; loopi < ciaaDriverKeyboardConst.countOfDevices; loopi++) {
      /* add each device */
      ciaaKeyboardDevices_addDriver(ciaaDriverKeyboardConst.devices[loopi]);
      /* init layer data for each device */
      *((ciaaDriverKeyboard_keybType *)ciaaDriverKeyboardConst.devices[loopi]->layer) = 0;
   }
}


/*==================[interrupt hanlders]=====================================*/

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

