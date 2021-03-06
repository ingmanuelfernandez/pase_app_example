/* Copyright 2017, Gustavo Muro
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

/** \brief PASE APP EXAMPLE
 **
 ** ejemplo de aplicación usando CIAA Firmware
 **
 **/

/** \addtogroup
 ** @{ */
/** \addtogroup
 ** @{ */
/** \addtogroup
 ** @{ */

/*==================[inclusions]=============================================*/
#include "os.h"
#include "pase_app_example.h"
#include "bsp.h"

/*==================[macros and definitions]=================================*/
#define FIRST_START_DELAY_MS 20
#define FIRST_START_DELAY_MS2 2500
#define WIDEPULSE1 1
#define PERIOD_MS 20
#define PERIOD_PULL 10
/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

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
   ShutdownOS(0);
}

/** \brief Initial task
 *
 * This task is started automatically in the application mode 1.
 */
TASK(InitTask)
{
   bsp_init();

   SetRelAlarm(ActivatePeriodicTask, FIRST_START_DELAY_MS, PERIOD_MS);

   //SetRelAlarm(ActivatePullingPulsEncTask, 0, PERIOD_PULL);
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

    bsp_ledAction(BOARD_LED_ID_0_R, BSP_LED_ACTION_ON);
	    SetRelAlarm(ActivateWidePulse1, WIDEPULSE1, WIDEPULSE1);

   TerminateTask();
}

TASK(EndActivateWidePulse1)
{

	bsp_ledAction(BOARD_LED_ID_0_R, BSP_LED_ACTION_OFF);
      CancelAlarm(ActivateWidePulse1);
   TerminateTask();
}
/*
TASK(PullingPulsEncTask)
{
	if (bsp_switchStatus(BOARD_TEC_ID_1))
	{
		SetRelAlarm(ActivatePrenderLedTask, FIRST_START_DELAY_MS2, PERIOD_MS);
		WaitEvent(LedApagado);
		GetEvent(ApagarLedTask, &LedApagado);
		CleraEvent(LedApagado);
	}
   TerminateTask();
}

TASK(PullingPulsOffTask)
{
	if (bsp_switchStatus(BOARD_TEC_ID_2))
	{
		SetRelAlarm(ActivateDelayLedOffTask, FIRST_START_DELAY_MS2, PERIOD_MS);
		WaitEvent(LedPrendido);
		GetEvent(PrenderLedTask, &LedPrendido);
		CleraEvent(LedPrendido);
	}
   TerminateTask();
}

TASK(PrenderLedTask)
{
    bsp_ledAction(BOARD_LED_ID_GREEN, BSP_LED_ACTION_ON);
	SetEvent(PullingPulsOffTask,LedPrendido);
   TerminateTask();
}

TASK(ApagarLedTask)
{
    bsp_ledAction(BOARD_LED_ID_GREEN, BSP_LED_ACTION_OFF);
	SetEvent(PullingPulsEncTask, LedApagado);
	TerminateTask();
}
*/


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

