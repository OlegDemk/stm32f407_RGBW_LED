/*
 * control.c
 *
 *  Created on: Nov 13, 2022
 *      Author: odemki
 */

#include "light/led_stript/ARGB.h"

// -----------------------------------------------------------------------------------------------------
void turn_off_left_stript(void)
{
	ARGB_Clear_left(); 			// Clear stirp
	while (ARGB_Show_left() != ARGB_OK); // Update - Option 1
}
// -----------------------------------------------------------------------------------------------------
void turn_off_right_stript(void)
{
	ARGB_Clear_right();
	while (ARGB_Show_right() != ARGB_OK); // Update - Option 1
}
// -----------------------------------------------------------------------------------------------------
void turn_off_left_and_right_dtript(void)
{
	turn_off_left_stript();
	turn_off_right_stript();
}
// -----------------------------------------------------------------------------------------------------
void turn_on_left_stript(u8_t r, u8_t g, u8_t b)
{
	uint8_t i = 0;
	for(i = 0; i<= 83; i++)
	{
		ARGB_SetRGB_left(i, r, g, b);
	}
	while (!ARGB_Show_left());  // Update
}
// -----------------------------------------------------------------------------------------------------
void turn_on_right_stript(u8_t r, u8_t g, u8_t b)
{
	uint8_t i = 0;
	for(i = 0; i<= 83; i++)
	{
		ARGB_SetRGB_right(i, r, g, b);
	}
	while (!ARGB_Show_right());  // Update
}
// -----------------------------------------------------------------------------------------------------
void turn_all_leds_from_centr(u8_t delay, u8_t r, u8_t g, u8_t b)
{
	uint8_t i =0;
	// turn_off_left_stript();

	for( i = 84; i >= 43; i--)
	{
		ARGB_SetRGB_left(i, r, g, b);			// High side
		ARGB_SetRGB_right(i, r, g, b);

		ARGB_SetRGB_left((85 - i), r, g, b);
		ARGB_SetRGB_right((85 - i), r, g, b);

		while (!ARGB_Show_left());  // Update
		while (!ARGB_Show_right());  // Update
		HAL_Delay(delay);
	}
}
// -----------------------------------------------------------------------------------------------------
void set_left_one_rgbw_led(uint8_t position, u8_t r, u8_t g, u8_t b, u8_t w)
{
	ARGB_SetRGB_left(position, r, g, b);
	ARGB_SetWhite_left(position, w);
}
// -----------------------------------------------------------------------------------------------------
void set_right_one_rgbw_led(uint8_t position, u8_t r, u8_t g, u8_t b, u8_t w)
{
	ARGB_SetRGB_right(position, r, g, b);
	ARGB_SetWhite_right(position, w);
}
// -----------------------------------------------------------------------------------------------------






