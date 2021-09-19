/*
 * ns_socket.h
 *
 *  Created on: 30.01.2021
 *      Author: joerg
 *
 *      Steckdose V1.1 / Mai 2020
 */

#ifndef MAIN_STECKDOSE_NS_SOCKET_H_
#define MAIN_STECKDOSE_NS_SOCKET_H_

#include "wiog_include/wiog_system.h"
#include "wiog_include/wiog_data.h"


extern void device_init();
extern void device_set_control(uint32_t bm);
extern void device_set_timer(uint32_t sek);
extern uint32_t device_get_out_bitmask();
extern uint32_t device_get_in_bitmask();

#endif /* MAIN_STECKDOSE_NS_SOCKET_H_ */
