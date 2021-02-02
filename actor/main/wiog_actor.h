/*
 * wiog_actor.h
 *
 *  Created on: 15.05.2020, Author: joerg
 *
 *  WIOG - Wireless Internet of Garden
 *  Tx-/Rx- Basisfunktionen - Actor/Repeater
 *
 */

#ifndef WIOG_ACTOR_H_
#define WIOG_ACTOR_H_

#define MIN_ACTOR_INTERVAL_MS 5*1000		//min 5Sek
#define MAX_ACTOR_INTERVAL_MS 60*60*1000	//max 1h
#define DEF_ACTOR_INTERVAL_MS 30*1000		//default 30sek

//öffentliche Variablen
extern uint8_t   wifi_channel;
extern uint16_t  cnt_no_response;
extern uint32_t  cycle;
extern uint32_t  interval_ms;
extern uint16_t  dev_uid; //Geräte-ID wird aus efuse_MAC berechnet


//öffentliche Funktionen
void wiog_actor_init(void);
bool send_data_frame(uint8_t* buf);
void wiog_set_channel(uint8_t ch);

//CallBack - Auswertung GW-Daten in main
typedef void(*wiog_rx_data_cb_t) (uint8_t* data);
void wiog_rx_register_cb(wiog_rx_data_cb_t pRx_data);

#endif /* WIOG_ACTOR_H_ */
