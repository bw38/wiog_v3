/*
 * wiog_wifi_actor.h
 *
 *  Created on: 06.06.2021
 *      Author: joerg
 */

#ifndef MAIN_WIOG_WIFI_ACTOR_H_
#define MAIN_WIOG_WIFI_ACTOR_H_

static uint8_t version = 0;
static uint8_t revision = 0;

extern uint8_t   wifi_channel;
extern uint32_t  cnt_no_response;
extern uint32_t  cnt_no_response_serie;
extern uint16_t my_uid; //Geräte-ID wird aus efuse_MAC berechnet
extern uint32_t interval_ms;
extern uint32_t cycles;
extern uint32_t onTime;

void (*cb_rx_ack_handler)(wiog_header_t* pHdr);
void (*cb_rx_data_handler)(wiog_header_t* pHdr, payload_t* pl, int len);

void wiog_wifi_actor_init();
void set_management_data (management_t* pMan);
void send_data_frame(payload_t* buf, uint16_t len, species_t spec);
void set_species(species_t sp);

#endif /* MAIN_WIOG_WIFI_ACTOR_H_ */
