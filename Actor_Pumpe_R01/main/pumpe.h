/*
 * pumpe.h
 *
 *  Created on: 30.01.2021
 *      Author: joerg
 *
 *  Steuerung Wasserpumpe Garten / Repeater Garage
 *
 */

#ifndef MAIN_PUMPE_PUMPE_H_
#define MAIN_PUMPE_PUMPE_H_

#include "../../wiog_include/wiog_system.h"
#include "../../wiog_include/wiog_data.h"

extern void device_init();
extern void device_set_control(uint32_t bm);
extern uint32_t device_get_in_bitmask();
extern uint32_t device_get_out_bitmask();


#endif /* MAIN_PUMPE_PUMPE_H_ */
