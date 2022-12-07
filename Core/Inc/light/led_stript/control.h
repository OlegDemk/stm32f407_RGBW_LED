/*
 * control.h
 *
 *  Created on: Nov 13, 2022
 *      Author: odemki
 */

#ifndef INC_LIGHT_LED_STRIPT_CONTROL_H_
#define INC_LIGHT_LED_STRIPT_CONTROL_H_

void turn_off_left_stript(void);
void turn_off_right_stript(void);
void turn_off_left_and_right_dtript(void);

void turn_on_left_stript(u8_t r, u8_t g, u8_t b);
void turn_on_right_stript(u8_t r, u8_t g, u8_t b);

void turn_all_leds_from_centr(u8_t delay, u8_t r, u8_t g, u8_t b);

void set_left_one_rgbw_led(uint8_t position, u8_t r, u8_t g, u8_t b, u8_t w);
void set_right_one_rgbw_led(uint8_t position, u8_t r, u8_t g, u8_t b, u8_t w);

#endif /* INC_LIGHT_LED_STRIPT_CONTROL_H_ */
