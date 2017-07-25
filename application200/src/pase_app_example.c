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

/*==================[inclusions]=============================================*/
#include "os.h"
#include "pase_app_example.h"
#include "bsp.h"

/*==================[macros and definitions]=================================*/
#define LOW_BRIGHTNESS 0
#define HIGH_BRIGHTNESS 7
#define FIRST_START_DELAY_MS 20
#define LED_BRIGHTNESS_PERIOD_MS 250 //(2 * 1000 / (HIGH_BRIGHTNESS - LOW_BRIGHTNESS +1))
#define KEY_ACTION_PERIOD_MS 2
#define KEY_READ_PERIOD_MS 2
#define LAST_LED 3
#define STATE_QUANTITY 3
#define KEY_QUANTITY 2

void F_Start_Sequence (void);

void F_Stop_Sequence (void);

void F_Pause_Sequence (void);

void F_Continue_Sequence (void);

void F_Nothing (void);

/*==================[internal data declaration]==============================*/
typedef enum {
	TASK_STATE_OFF = 0,
	TASK_STATE_ON,
	TASK_STATE_PAUSE,
	TASK_STATE_QUANTITY
} task_state_enum;

typedef enum {
	TASK_RAMP_RISING = 0,
	TASK_RAMP_FALLING,
} task_ramp_enum;

/*==================[internal data definition]===============================*/
static int32_t key_press = NONE_KEY;
static board_ledId_enum ledId = BOARD_LED_ID_0_R;
static task_ramp_enum task_ramp = TASK_RAMP_RISING;
static int32_t led_brightness_level = LOW_BRIGHTNESS;
static task_state_enum task_state = TASK_STATE_ON;


/*
 * MAQUINA DE ESTADO DE CONTROL DE SECUENCIA DE BRILLO DE LEDS
 */
const action_key_type action_key[BOARD_TEC_ID_QUANTITY][TASK_STATE_QUANTITY] = {
		/* BOARD_TEC_ID_1   BOARD_TEC_ID_2     */
		{{F_Start_Sequence },{F_Nothing}},				/* TASK_STATE_OFF   */
		{{F_Stop_Sequence} ,{F_Pause_Sequence}},	/* TASK_STATE_ON    */
		{{F_Stop_Sequence} ,{F_Continue_Sequence}}	/* TASK_STATE_PAUSE */
};





/*==================[internal functions definition]==========================*/

void Update_Brightness(int32_t brightness_level)
{
    bsp_pwmSetDutyCycle(brightness_level);
}

void Current_Led_Off(void)
{
	bsp_ledAction(ledId, BSP_LED_ACTION_OFF);
}

int32_t Read_Key(void)
{
	return bsp_keyboardGet();
}

void Switch_Led(void)
{
	ledId++;
	if (ledId == LAST_LED) ledId = BOARD_LED_ID_0_R;
	bsp_pwmSelectLed(ledId);
}

void ErrorHook(void)
{
   ShutdownOS(0);
}

/*================== [     ACCIONES DE TECLADO     ] =========================*/

void F_Nothing (void)
{
	return;
}

void F_Start_Sequence (void)
{
// OJO inhibir interrupcion de Timer
	ledId = BOARD_LED_ID_0_R;
	task_ramp = TASK_RAMP_RISING;
	led_brightness_level = LOW_BRIGHTNESS;
	task_state = TASK_STATE_ON;

	bsp_pwmSelectLed(ledId);
	bsp_pwmStart();
    SetRelAlarm(ActivatePeriodicLedBrightness, FIRST_START_DELAY_MS, LED_BRIGHTNESS_PERIOD_MS);
// Habilitar interrupcion de Timer
	return;
}

void F_Stop_Sequence (void)
{
	// OJO inhibir interrupcion de Timer o detener el Timer
	bsp_pwmStop();
	Current_Led_Off();
	if (task_state != TASK_STATE_PAUSE)
			CancelAlarm(ActivatePeriodicLedBrightness);
	task_state = TASK_STATE_OFF;
	return;
}

void F_Pause_Sequence (void)
{
	// OJO inhibir interrupcion de Timer
		task_state = TASK_STATE_PAUSE;
		CancelAlarm(ActivatePeriodicLedBrightness);
	// Habilitar interrupcion de Timer

	return;
}

void F_Continue_Sequence (void)
{
	// OJO inhibir interrupcion de Timer
		task_state = TASK_STATE_ON;
	    SetRelAlarm(ActivatePeriodicLedBrightness, FIRST_START_DELAY_MS, LED_BRIGHTNESS_PERIOD_MS);
	// Habilitar interrupcion de Timer
	return;
}


/*=================== [       FUNCION MAIN       ] ==========================*/

int main(void)
{
   StartOS(AppMode1);
   return 0;
}

/*================== [           TAREAS           ] =========================*/

TASK(InitTask)
{
   bsp_init();
   SetRelAlarm(ActivatePeriodicLedBrightness, FIRST_START_DELAY_MS, LED_BRIGHTNESS_PERIOD_MS);
//   SetRelAlarm(ActivatePeriodicKeyAction, FIRST_START_DELAY_MS, KEY_ACTION_PERIOD_MS);
   SetRelAlarm(ActivatePeriodicKeyRead, FIRST_START_DELAY_MS, KEY_READ_PERIOD_MS);
   ActivateTask(KeyAction);
   TerminateTask();
}


/*TASK(PeriodicKeyAction)
{
	int32_t key_press;
	void (*f_action_key)(void);
//	action_key_type f_action_key;
	key_press = Read_Key();
	if (key_press != NONE_KEY)
	{
		f_action_key = action_key[task_state][key_press].f;
		f_action_key();
	}
	TerminateTask();
}
*/
TASK(KeyAction)
{
	void (*f_action_key)(void);
	while (1)
	{
		WaitEvent(evKeyPress);
		ClearEvent(evKeyPress);
		f_action_key = action_key[task_state][key_press].f;
		f_action_key();
	}
}

TASK(PeriodicKeyRead)
{
	bsp_keyboard_task();
	key_press = Read_Key();
	if (key_press != NONE_KEY)
	{
		SetEvent(KeyAction,evKeyPress);
	}
	TerminateTask();
}


TASK(PeriodicLedBrightness)
{
    if (task_ramp == TASK_RAMP_RISING)
    {
    	if (led_brightness_level == HIGH_BRIGHTNESS)
    	{
    		task_ramp =TASK_RAMP_FALLING;
    	//	led_brightness_level --;
    	}
    	else
        	led_brightness_level ++;
    }
    else
    {
    	if (led_brightness_level == LOW_BRIGHTNESS)
    	{
    		task_ramp = TASK_RAMP_RISING;
    		Current_Led_Off();
    		Switch_Led();
    	}
    	else
        	led_brightness_level --;
    }

    Update_Brightness(led_brightness_level);
    TerminateTask();
}

/*==================[end of file]============================================*/
