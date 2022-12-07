/*
 * stop_light.c
 *
 *  Created on: Oct 24, 2022
 *      Author: odemki
 */
#include "main.h"

#include "light/led_stop/stop_light.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/*
						Map of STOP RED LEDs:
	LEFT SIDE                                                RIGHT SIDE
	-------------------------------------------------------------------
	EFT LED5  LED4  LED3  LED2  LED1		LED1  LED2  LED3  LED4 LED5
	-------------------------------------------------------------------
 */
#define MAX_DEMO_BRIGHTNESS 100

#define LEFT_1_ON		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1)
#define LEFT_1_OFF		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1)
#define LEFT_2_ON 		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2)
#define LEFT_2_OFF		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2)
#define LEFT_3_ON		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3)
#define LEFT_3_OFF		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3)
#define LEFT_4_ON		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4)
#define LEFT_4_OFF		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4)
#define LEFT_5_ON		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1)
#define LEFT_5_OFF		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1)

#define RIGHT_1_ON		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2)
#define RIGHT_1_OFF		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2)
#define RIGHT_2_ON		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3)
#define RIGHT_2_OFF		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3)
#define RIGHT_3_ON		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4)
#define RIGHT_3_OFF		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4)
#define RIGHT_4_ON		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2)
#define RIGHT_4_OFF		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2)
#define RIGHT_5_ON		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3)
#define RIGHT_5_OFF		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3)


// ----------------------------------------------------------------------------------------
void stop_light_turn_on_left(void)
{
	LEFT_1_ON; 		LEFT_2_ON;		LEFT_3_ON;		LEFT_4_ON;		LEFT_5_ON;
}

// ----------------------------------------------------------------------------------------
void stop_light_turn_on_right(void)
{
	RIGHT_1_ON;		RIGHT_2_ON;		RIGHT_3_ON;		RIGHT_4_ON;		RIGHT_5_ON;
}

// ----------------------------------------------------------------------------------------
void stop_light_turn_off_left(void)
{
	LEFT_1_OFF;		LEFT_2_OFF;		LEFT_3_OFF;		LEFT_4_OFF;		LEFT_5_OFF;
}

// ----------------------------------------------------------------------------------------
void stop_light_turn_off_right(void)
{
	RIGHT_1_OFF;	RIGHT_2_OFF;	RIGHT_3_OFF;	RIGHT_4_OFF;	RIGHT_5_OFF;
}

// ----------------------------------------------------------------------------------------
void stop_light_turn_on_from_center(int delay)
{
	LEFT_5_ON;
	RIGHT_5_ON;
	HAL_Delay(delay);

	LEFT_4_ON;
	RIGHT_4_ON;
	HAL_Delay(delay);

	LEFT_3_ON;
	RIGHT_3_ON;
	HAL_Delay(delay);

	LEFT_2_ON;
	RIGHT_2_ON;
	HAL_Delay(delay);

	LEFT_1_ON;
	RIGHT_1_ON;
	HAL_Delay(delay);
}
// ----------------------------------------------------------------------------------------
void set_duty_cycle_stop_left_1(int duty_cycle)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_cycle);
}
// ----------------------------------------------------------------------------------------
void set_duty_cycle_stop_left_2(int duty_cycle)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty_cycle);
}
// ---------------------------------------------------------------------------------------
void set_duty_cycle_stop_left_3(int duty_cycle)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_cycle);
}
// ---------------------------------------------------------------------------------------
void set_duty_cycle_stop_left_4(int duty_cycle)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty_cycle);
}
// ----------------------------------------------------------------------------------------
void set_duty_cycle_stop_left_5(int duty_cycle)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle);
}
// ----------------------------------------------------------------------------------------
void set_duty_cycle_stop_ritht_1(int duty_cycle)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, duty_cycle);
}
// ----------------------------------------------------------------------------------------
void set_duty_cycle_stop_ritht_2(int duty_cycle)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, duty_cycle);
}
// ---------------------------------------------------------------------------------------
void set_duty_cycle_stop_ritht_3(int duty_cycle)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty_cycle);
}
// ---------------------------------------------------------------------------------------
void set_duty_cycle_stop_ritht_4(int duty_cycle)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_cycle);
}
// ----------------------------------------------------------------------------------------
void set_duty_cycle_stop_ritht_5(int duty_cycle)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_cycle);
}
// ----------------------------------------------------------------------------------------
void stop_light_all_turn_on(void)
{
	set_duty_cycle_stop_left_1(255);
	set_duty_cycle_stop_left_2(250);
	set_duty_cycle_stop_left_3(250);
	set_duty_cycle_stop_left_4(250);
	set_duty_cycle_stop_left_5(250);

	set_duty_cycle_stop_ritht_1(250);
	set_duty_cycle_stop_ritht_2(250);
	set_duty_cycle_stop_ritht_3(250);
	set_duty_cycle_stop_ritht_4(250);
	set_duty_cycle_stop_ritht_5(250);

	  // Turn on RED LEDs
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	stop_light_turn_on_left();
	stop_light_turn_on_right();
}

// ----------------------------------------------------------------------------------------
void stop_light_all_turn_off(void)
{

	stop_light_turn_off_left();
	stop_light_turn_off_right();
}

// ----------------------------------------------------------------------------------------
void test_changes_bratiles(void)
{
	int delay = 3;

	  // Turn on RED LEDs
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	for(int duty_cycle = 0; duty_cycle <= 250; duty_cycle++)
	{
		set_duty_cycle_stop_left_1(duty_cycle);
		set_duty_cycle_stop_left_2(duty_cycle);
		set_duty_cycle_stop_left_3(duty_cycle);
		set_duty_cycle_stop_left_4(duty_cycle);
		set_duty_cycle_stop_left_5(duty_cycle);

		set_duty_cycle_stop_ritht_1(duty_cycle);
		set_duty_cycle_stop_ritht_2(duty_cycle);
		set_duty_cycle_stop_ritht_3(duty_cycle);
		set_duty_cycle_stop_ritht_4(duty_cycle);
		set_duty_cycle_stop_ritht_5(duty_cycle);

		stop_light_turn_on_left();
		stop_light_turn_on_right();

		HAL_Delay(delay);

		if(duty_cycle == 250)
		{
			for(duty_cycle = 250; duty_cycle >= 0; duty_cycle--)
			{
				set_duty_cycle_stop_left_1(duty_cycle);
				set_duty_cycle_stop_left_2(duty_cycle);
				set_duty_cycle_stop_left_3(duty_cycle);
				set_duty_cycle_stop_left_4(duty_cycle);
				set_duty_cycle_stop_left_5(duty_cycle);

				set_duty_cycle_stop_ritht_1(duty_cycle);
				set_duty_cycle_stop_ritht_2(duty_cycle);
				set_duty_cycle_stop_ritht_3(duty_cycle);
				set_duty_cycle_stop_ritht_4(duty_cycle);
				set_duty_cycle_stop_ritht_5(duty_cycle);

				stop_light_turn_on_left();
				stop_light_turn_on_right();

				HAL_Delay(delay);
			}
		}
	}
}
// ----------------------------------------------------------------------------------------
void test_from_midle_to_corner(void)
{
	int delay = 1;
	int duty_cycle = 0;

	  // Turn on RED LEDs
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	for(duty_cycle = 0; duty_cycle <= MAX_DEMO_BRIGHTNESS; duty_cycle++)
	{
		set_duty_cycle_stop_left_1(duty_cycle);
		set_duty_cycle_stop_ritht_1(duty_cycle);

		stop_light_turn_on_left();
		stop_light_turn_on_right();

		HAL_Delay(delay);
	}

	for(duty_cycle = 0; duty_cycle <= MAX_DEMO_BRIGHTNESS; duty_cycle++)
	{
		set_duty_cycle_stop_left_2(duty_cycle);
		set_duty_cycle_stop_ritht_2(duty_cycle);

		stop_light_turn_on_left();
		stop_light_turn_on_right();

		HAL_Delay(delay);
	}

	for(duty_cycle = 0; duty_cycle <= MAX_DEMO_BRIGHTNESS; duty_cycle++)
	{
		set_duty_cycle_stop_left_3(duty_cycle);
		set_duty_cycle_stop_ritht_3(duty_cycle);

		stop_light_turn_on_left();
		stop_light_turn_on_right();

		HAL_Delay(delay);
	}

	for(duty_cycle = 0; duty_cycle <= MAX_DEMO_BRIGHTNESS; duty_cycle++)
	{
		set_duty_cycle_stop_left_4(duty_cycle);
		set_duty_cycle_stop_ritht_4(duty_cycle);

		stop_light_turn_on_left();
		stop_light_turn_on_right();

		HAL_Delay(delay);
	}

	for(duty_cycle = 0; duty_cycle <= MAX_DEMO_BRIGHTNESS; duty_cycle++)
	{
		set_duty_cycle_stop_left_5(duty_cycle);
		set_duty_cycle_stop_ritht_5(duty_cycle);

		stop_light_turn_on_left();
		stop_light_turn_on_right();

		HAL_Delay(delay);
	}

	for(duty_cycle = MAX_DEMO_BRIGHTNESS; duty_cycle >= 0; duty_cycle--)
	{
		set_duty_cycle_stop_left_1(duty_cycle);
		set_duty_cycle_stop_left_2(duty_cycle);
		set_duty_cycle_stop_left_3(duty_cycle);
		set_duty_cycle_stop_left_4(duty_cycle);
		set_duty_cycle_stop_left_5(duty_cycle);

		set_duty_cycle_stop_ritht_1(duty_cycle);
		set_duty_cycle_stop_ritht_2(duty_cycle);
		set_duty_cycle_stop_ritht_3(duty_cycle);
		set_duty_cycle_stop_ritht_4(duty_cycle);
		set_duty_cycle_stop_ritht_5(duty_cycle);

		stop_light_turn_on_left();
		stop_light_turn_on_right();

		HAL_Delay(delay);
	}

}














