/* Copyright 2017, Gustavo Muro
 * All rights reserved.
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

/** \brief source para MCU
 **
 ** archivo de inicilización del microcontrolador
 **
 **/

/** \addtogroup PASE_APP_EXAMPLE
 ** @{ */
/** \addtogroup MCU
 ** @{ */

/*==================[inclusions]=============================================*/
#include "mcu.h"
#include "stdint.h"
#include "chip.h"
#include "os.h"
#include "mcu_gpio.h"
/*==================[macros and definitions]=================================*/
#define ON 1
#define OFF 0

/*==================[internal data declaration]==============================*/

static mcu_gpio_pinId_enum pin;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
extern void mcu_pwm_init(void)
{
	   /* Timer */
   Chip_TIMER_Init(LPC_TIMER1);
   Chip_TIMER_PrescaleSet(LPC_TIMER1,
	#ifdef lpc1769
	         Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_TIMER1) / 1000000 - 1
	#else
			 Chip_Clock_GetRate(CLK_MX_TIMER1) / 1000000 - 1
	#endif
	   );

	}

extern void mcu_pwm_config(void)
{
	   /* Match 0 (period) */
	   Chip_TIMER_MatchEnableInt(LPC_TIMER1, 0);
	   Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1, 0);
	   Chip_TIMER_StopOnMatchDisable(LPC_TIMER1, 0);
	   Chip_TIMER_SetMatch(LPC_TIMER1, 0, 1000);

	   /* Match 1 (duty) */
	   Chip_TIMER_MatchEnableInt(LPC_TIMER1, 1);
	   Chip_TIMER_ResetOnMatchDisable(LPC_TIMER1, 1);
	   Chip_TIMER_StopOnMatchDisable(LPC_TIMER1, 1);
	   Chip_TIMER_SetMatch(LPC_TIMER1, 1, 100);

	   Chip_TIMER_Reset(LPC_TIMER1);
	   Chip_TIMER_Enable(LPC_TIMER1);
//	   Chip_TIMER_Disable(LPC_TIMER1);
	   NVIC_EnableIRQ(TIMER1_IRQn);
   }

extern void mcu_pwm_stop (void)
{
	Chip_TIMER_Disable(LPC_TIMER1);
}

extern void mcu_pwm_start (void)
{
	Chip_TIMER_Reset(LPC_TIMER1);
	Chip_TIMER_Enable(LPC_TIMER1);
}

extern void mcu_pwm_selectPin(mcu_gpio_pinId_enum id)
{
	pin = id;
}

extern void mcu_pwm_setDutyCycle(uint32_t duty)
{
	  Chip_TIMER_SetMatch(LPC_TIMER1, 1, duty);
	  Chip_TIMER_Reset(LPC_TIMER1);
	  Chip_TIMER_ClearMatch(LPC_TIMER1, 1);
	  Chip_TIMER_ClearMatch(LPC_TIMER1, 0);


}

ISR(TIMER1_Interrupt)
//void TIMER1_IRQHandler(void)
{
   if (Chip_TIMER_MatchPending(LPC_TIMER1, 0)) {
      Chip_TIMER_ClearMatch(LPC_TIMER1, 0);
      mcu_gpio_setOut(pin,ON);
   }
   if (Chip_TIMER_MatchPending(LPC_TIMER1, 1)) {
      Chip_TIMER_ClearMatch(LPC_TIMER1, 1);
      mcu_gpio_setOut(pin, OFF);
   }
}



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
