/*
 * wiog_sen.h
 *
 *  Created on: 05.05.2020, Author: joerg
 *
 *  WIOG - Wireless Internet of Garden
 *  Tx-/Rx- Basisfunktionen - Sensor
 *
 */

#ifndef WIOG_SEN_H_
#define WIOG_SEN_H_

//öffentliche Variablen
RTC_DATA_ATTR static uint32_t rtc_onTime;
RTC_DATA_ATTR static uint8_t  rtc_no_response_cnt;


//Daten, die einen Deep-Sleep überstehen müssen
RTC_DATA_ATTR static uint8_t  rtc_wifi_channel;
RTC_DATA_ATTR static uint32_t rtc_cycles;
RTC_DATA_ATTR static uint32_t rtc_interval_ms;
RTC_DATA_ATTR static int8_t   rtc_tx_pwr;		//Steuerung Sendeleistung
RTC_DATA_ATTR static uint32_t rtc_cnt_no_scan;  //Fehlversuche Channelscan

//öffentliche Funktionen
bool wiog_sensor_init();
bool send_data_frame(uint8_t* buf);

#endif /* WIOG_SEN_H_ */
