/*
 * gw_main.h
 *
 *  Created on: 21.02.2021
 *      Author: joerg
 */

#ifndef MAIN_GW_MAIN_H_
#define MAIN_GW_MAIN_H_

#include "driver/gpio.h"

#include "wiog_include/wiog_system.h"
#include "wiog_include/wiog_data.h"
#include "shotlist.h"


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

//Arbeitskanel, Vorgabe durch RPi im c-Frame
extern uint8_t wifi_channel;

//statisches Array mit Default- und Geräteinformationen
//Daten werden von RPi im d-Frame bereitgestellt
extern device_info_block_t dib;


//Prioritäten und Slots (synchron in allen Nodes (incl gw))
extern node_info_block_t nib;

//Liste der (einmalig)zu resettenden UID
extern shotlist_t sl_uid_reset;


void send_data_frame(payload_t* buf, uint16_t len, dev_uid_t uid);
void broadcast_nib(node_info_block_t* pnib);
device_info_t* get_device_info(dev_uid_t uid);
void notice_payload(dev_uid_t uid, uint8_t x);

#endif /* MAIN_GW_MAIN_H_ */
