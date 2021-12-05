/*
 * wiog_wifi.h
 *
 *  Created on: 26.05.2021
 *      Author: joerg
 */

#ifndef WIOG_WIFI_SENSOR_H_
#define WIOG_WIFI_SENSOR_H_

//static uint8_t version = 0;
//static uint8_t revision = 0;

extern uint16_t my_uid; //Geräte-ID wird aus efuse_MAC berechnet
extern uint32_t rtc_interval_ms;
extern uint32_t rtc_cycles;
extern uint32_t rtc_onTime;

void (*cb_rx_handler)(wiog_header_t* pHdr);

void wiog_wifi_sensor_init();
void set_management_data (management_t* pMan);
void send_data_frame(payload_t* buf, uint16_t len);
void wiog_wifi_sensor_goto_sleep(wakeup_src_t wus);


#endif /* WIOG_WIFI_SENSOR_H_ */
