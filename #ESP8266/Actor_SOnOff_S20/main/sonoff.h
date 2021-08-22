/*
 * sonoff.h
 *
 *  Created on: 02.08.2021
 *      Author: joerg
 */

#ifndef MAIN_SONOFF_H_
#define MAIN_SONOFF_H_


extern void device_init();
extern void device_set_ns_state(int x);
extern void device_set_ns_timer(int sek);
extern int32_t device_get_ns_state();
extern int32_t device_get_button_state();
extern void device_set_status_led(int x);


#endif /* MAIN_SONOFF_H_ */
