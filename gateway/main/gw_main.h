/*
 * gw_main.h
 *
 *  Created on: 21.02.2021
 *      Author: joerg
 */

#ifndef MAIN_GW_MAIN_H_
#define MAIN_GW_MAIN_H_

#include "driver/gpio.h"

#include "../../wiog_include/wiog_system.h"
#include "../../wiog_include/wiog_data.h"


//Status-LED
#define LED_BL			GPIO_NUM_25
#define LED_STATUS_ON	gpio_set_level(LED_BL, 1)
#define LED_STATUS_OFF	gpio_set_level(LED_BL, 0)
#define LED_BL_TOGGLE	gpio_set_level(LED_BL, !gpio_get_level(LED_BL))

#define LED_GN			GPIO_NUM_26
#define LED_GN_ON		gpio_set_level(LED_GN, 1)
#define LED_GN_OFF		gpio_set_level(LED_GN, 0)
#define LED_GN_TOGGLE	gpio_set_level(LED_GN, !gpio_get_level(LED_GN))

#define LED_RT			GPIO_NUM_27
#define LED_RT_ON		gpio_set_level(LED_RT, 1)
#define LED_RT_OFF		gpio_set_level(LED_RT, 0)
#define LED_RT_TOGGLE	gpio_set_level(LED_RT, !gpio_get_level(LED_RT))

//statisches Array mit Default- und Geräteinformationen
//Daten werden von RPi im d-Frame bereitgestellt
device_info_block_t dib;


//Prioritäten und Slots (synchron in allen Nodes (incl gw))
node_info_block_t nib;

void broadcast_nib(node_info_block_t* pnib);

#endif /* MAIN_GW_MAIN_H_ */
