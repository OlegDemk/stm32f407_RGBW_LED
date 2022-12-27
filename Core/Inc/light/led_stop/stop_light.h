/*
 * stop_light.h
 *
 *  Created on: Oct 24, 2022
 *      Author: odemki
 */

#ifndef INC_LIGHT_LED_STOP_STOP_LIGHT_H_
#define INC_LIGHT_LED_STOP_STOP_LIGHT_H_

void stop_light_all_turn_on(void);
void stop_light_all_turn_off(void);
void stop_light_turn_on_left(void);
void stop_light_turn_on_right(void);
void stop_light_turn_off_left(void);
void stop_light_turn_off_right(void);
void stop_light_turn_on_from_center(int delay);

void set_duty_cycle_stop_left_1(int duty_cycle);
void set_duty_cycle_stop_left_2(int duty_cycle);
void set_duty_cycle_stop_left_3(int duty_cycle);
void set_duty_cycle_stop_left_4(int duty_cycle);
void set_duty_cycle_stop_left_5(int duty_cycle);
void set_duty_cycle_stop_ritht_1(int duty_cycle);
void set_duty_cycle_stop_ritht_2(int duty_cycle);
void set_duty_cycle_stop_ritht_3(int duty_cycle);
void set_duty_cycle_stop_ritht_4(int duty_cycle);
void set_duty_cycle_stop_ritht_5(int duty_cycle);

void test_changes_bratiles(void);
void test_from_midle_to_corner(void);


#endif /* INC_LIGHT_LED_STOP_STOP_LIGHT_H_ */
