/*
 * wiog_wifi_sensor.c
 *
 *  Created on: 26.05.2021
 *      Author: joerg
 */

#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "string.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp32/ulp.h"

#include "wiog_system.h"
#include "wiog_data.h"
#include "wiog_wifi_sensor.h"


#undef  LOG_LOCAL_LEVEL	//Warnhinweis vermeiden
#define LOG_LOCAL_LEVEL ESP_LOG_NONE	//s. readme.txt

//Daten, die einen Deep-Sleep überstehen müssen
RTC_DATA_ATTR uint8_t  rtc_wifi_channel;
RTC_DATA_ATTR uint32_t rtc_cycles;
RTC_DATA_ATTR uint32_t rtc_interval_ms;
RTC_DATA_ATTR int8_t   rtc_tx_pwr;		//Steuerung Sendeleistung
RTC_DATA_ATTR uint32_t rtc_cnt_no_scan;  //Fehlversuche Channelscan
RTC_DATA_ATTR uint32_t rtc_no_response;  //Fehlversuche Übertragung
RTC_DATA_ATTR uint8_t  rtc_no_response_serie; //Fehlversuche aufeinanderfolgend
RTC_DATA_ATTR uint32_t rtc_onTime;		//Kumulation Prozessorzeit


uint32_t ack_id = 0;		//Vergleich mit Tx-FrameID
uint32_t tx_fid;
uint32_t interval_ms;
int8_t tx_pwr_delta_dB = 0;
dev_uid_t my_uid;

SemaphoreHandle_t ack_timeout_Semaphore = NULL;
SemaphoreHandle_t goto_sleep_Semaphore = NULL;

#define WIOG_RX_QUEUE_SIZE 12
xQueueHandle wiog_rx_queue;

#define WIOG_TX_QUEUE_SIZE 6
xQueueHandle wiog_tx_queue;



//Wifi-Rx-Callback im Sniffermode - Daten in die Rx-Queue stellen
IRAM_ATTR void wiog_receive_packet_cb(void* buff, wifi_promiscuous_pkt_type_t type)
{
	const wifi_promiscuous_pkt_t   *ppkt = (wifi_promiscuous_pkt_t *)buff;
	const wiog_data_frame_t  *ipkt = (wiog_data_frame_t *)ppkt->payload;
	const wiog_header_t *pHdr =  &ipkt->header;

	//nur fehlerfreie Pakete des eigenen Netzes bearbeiten
	if ((ppkt->rx_ctrl.rx_state != 0) || (memcmp(pHdr->mac_net, &mac_net, sizeof(mac_addr_t)) !=0)) return;
	//nur per UID adressierte Pakete akzeptieren
	if (pHdr->uid != my_uid) return;
	//Pakettypen filtern
	if ((pHdr->vtype == ACK_FOR_CHANNEL) || (pHdr->vtype == ACK_FROM_GW)) {
		wiog_event_rxdata_t frame;
		memcpy(&frame.rx_ctrl, &ppkt->rx_ctrl, sizeof(wifi_pkt_rx_ctrl_t));
		memcpy(&frame.wiog_hdr, pHdr, sizeof(wiog_header_t));
		frame.data_len = ppkt->rx_ctrl.sig_len - sizeof(wiog_header_t) - 4; //ohne FCS
		frame.pdata = (uint8_t*)malloc(frame.data_len);
		memcpy(frame.pdata, &ipkt->pdata, frame.data_len);

		//Event in die Rx-Queue stellen
		if (xQueueSend(wiog_rx_queue, &frame, portMAX_DELAY) != pdTRUE) {
			ESP_LOGW("Rx_Quue", "receive queue fail");
			free(frame.pdata);
		}
	}
}

//verzögertes Senden von Datenpaketen (Node-Slots)
void cb_tx_delay_slot(void* arg) {  //one-shot-timer
	wiog_event_txdata_t* ptx_frame = arg;
	if (xQueueSend(wiog_tx_queue, ptx_frame, portMAX_DELAY) != pdTRUE) {
			ESP_LOGW("Tx-Queue: ", "Error");
	}
	free(ptx_frame);
}


//Verarbeitung eines empfangenen Datenpaketes
IRAM_ATTR static void wiog_rx_processing_task(void *pvParameter)
{
	wiog_event_rxdata_t evt;

	while ((xQueueReceive(wiog_rx_queue, &evt, portMAX_DELAY) == pdTRUE)) {

//		wifi_pkt_rx_ctrl_t *pRx_ctrl = &evt.rx_ctrl;
		wiog_header_t *pHdr = &evt.wiog_hdr;

		// Antwort auf Channel-Scan ------------------------------------------
		if ((rtc_wifi_channel == 0) && (pHdr->vtype == ACK_FOR_CHANNEL))
		{
			rtc_wifi_channel = pHdr->channel;	//Arbeitskanal wird in jedem WIOG-Header geliefert
		}

		else
		// ACK des GW auf einen Datenframe, Tx-Wiederholungen stoppen
		if ((pHdr->vtype == ACK_FROM_GW) && (tx_fid == pHdr->frameid)) {
			//Interval-Info - Plausibilitätsprüfung vor Deep_Sleep
			interval_ms = pHdr->interval_ms;
			//bei Kanal-Abweichung Channel-Scan veranlassen
			if (rtc_wifi_channel != pHdr->channel) rtc_wifi_channel = 0;
			//empfohlene Tx-Pwr-Korrektur
			tx_pwr_delta_dB = pHdr->txpwr;

			//Wiederholung stoppen
//			tx_fid = 0;
			xSemaphoreGive(ack_timeout_Semaphore);

			//Callback sensor_main
			(*cb_rx_handler)(pHdr);

			//Mainloop fortsetzen
			xSemaphoreGive(goto_sleep_Semaphore);
		}

		free(evt.pdata);
	}	// Rx-Queue
}


//Senden eines Datenpaketes
IRAM_ATTR void wiog_tx_processing_task(void *pvParameter) {

	wiog_event_txdata_t evt;

	while ((xQueueReceive(wiog_tx_queue, &evt, portMAX_DELAY) == pdTRUE)) {

		uint16_t tx_len = get_blocksize(evt.data_len) + sizeof(wiog_header_t);
		uint8_t buf[tx_len];
		bzero(buf, tx_len);
		//Header in Puffer kopieren
		memcpy(buf, &evt.wiog_hdr, sizeof(wiog_header_t));

		if (evt.crypt_data) {
/*
			//Datenblock verschlüsseln
			//CBC-AES-Key
			uint8_t key[] = {AES_KEY};
			// Frame-ID => 4 letzten Byres im Key
			uint32_t u32 = evt.wiog_hdr.frameid;
			key[28] = (uint8_t)u32;
			key[29] = (uint8_t)(u32>>=8);
			key[30] = (uint8_t)(u32>>=8);
			key[31] = (uint8_t)(u32>>=8);
			cbc_encrypt(evt.pdata, &buf[sizeof(wiog_header_t)], evt.data_len, key, sizeof(key));
*/
			wiog_encrypt_data(evt.pdata, &buf[sizeof(wiog_header_t)], evt.data_len, evt.wiog_hdr.frameid);
		} else {
			memcpy(&buf[sizeof(wiog_header_t)], evt.pdata, evt.data_len);
		}

		//Semaphore resetten
		xSemaphoreTake(ack_timeout_Semaphore, 0);
		((wiog_header_t*) buf)->seq_ctrl = 0;
		//max Wiederholungen bis ACK von GW oder Node
		tx_fid = evt.wiog_hdr.frameid;
		for (int i = 0; i <= evt.tx_max_repeat; i++) {
			//ab 3.Wiederholung mit voller Leistung senden.
			if (i > 2) {
				esp_wifi_set_max_tx_power(MAX_TX_POWER);
				tx_pwr_delta_dB = 6;	//next cycle mit höherer Tx-Leistung
			}
			//Frame senden
			esp_wifi_80211_tx(WIFI_IF_STA, buf, tx_len, false);
			if (evt.tx_max_repeat == 0) break;	//1x Tx ohne ACK
			//warten auf Empfang eines ACK
			if (xSemaphoreTake(ack_timeout_Semaphore, TX_REPEAT_INTERVAL) == pdTRUE) {
				break; //ACK empfangen -> Wiederholung abbrechen
			}

			((wiog_header_t*) buf)->seq_ctrl++ ;	//Sequence++
		}
		free(evt.pdata);

	}
}

// Datenverarbeitung ----------------------------------------------------------------------------

//Eintragen der Management-Daten in den Payload
//Aufrufer aktualisiert später die Anzahl der Datensätze
//Prefix des verschlüsselten Datenblocks an RPi
void set_management_data (management_t* pMan) {
	pMan->crc16 = 0;
	pMan->len = 0;
	pMan->uid = my_uid;
	pMan->wifi_channel = rtc_wifi_channel;
	pMan->species = SENSOR;
	pMan->version = version;
	pMan->revision = revision;
	pMan->cnt_no_response = rtc_no_response;
	ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&pMan->tx_pwr));
	pMan->cnt_entries = 0;
}


//Datenframe managed an Gateway senden
void send_data_frame(payload_t* buf, uint16_t len) {

	//unverschlüsselten Payload absichern
	buf->man.len = len;
	buf->man.crc16 = crc16((uint8_t*)&buf->man.len, len-2);

	uint8_t *data = (uint8_t*)buf;	//Datenbereich
	wiog_event_txdata_t tx_frame;
	tx_frame.wiog_hdr = wiog_get_dummy_header(GATEWAY, SENSOR);
	tx_frame.wiog_hdr.uid = my_uid;
	tx_frame.wiog_hdr.species = SENSOR;
	tx_frame.wiog_hdr.channel = rtc_wifi_channel;
	tx_frame.wiog_hdr.vtype = DATA_TO_GW;
	tx_frame.wiog_hdr.frameid = esp_random();
	tx_frame.wiog_hdr.tagD = 0;
	tx_frame.tx_max_repeat = TX_REPEAT_CNT_MAX;	//max Wiederholungen, ACK erwartet

	tx_frame.pdata = malloc(len);
	memcpy(tx_frame.pdata, data, len);
	tx_frame.data_len = len;
	tx_frame.crypt_data = true;
	tx_frame.target_time = 0;

	if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE)
		ESP_LOGW("Tx-Queue: ", "Tx Data fail");

}

//WIOG-Kanal setzen
//Kanal == 0 : Scann nach Arbeitskanal
//Scan erfolglos: DeepSleep
//Ziel des Scan ist immer der Gateway (wird auch von Repeatern ausgewertet)
void wiog_set_channel(uint8_t ch) {
	if (ch == 0) {
		wiog_event_txdata_t tx_frame = {
			.crypt_data = false,
			.data_len = 0,
			.pdata = NULL,
			.target_time = 0	//sofort senden
		};
		tx_frame.wiog_hdr = wiog_get_dummy_header(GATEWAY, SENSOR);
		tx_frame.wiog_hdr.vtype = SCAN_FOR_CHANNEL;
		tx_frame.wiog_hdr.uid = my_uid;
		tx_frame.wiog_hdr.species = SENSOR;
		tx_frame.wiog_hdr.frameid = 0;
		tx_frame.h_timer = NULL;

		rtc_tx_pwr = MAX_TX_POWER;
		ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(rtc_tx_pwr));

		for (ch = 1; ch <= wifi_country_de.nchan; ch++) {
			ESP_ERROR_CHECK( esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE));
			//Tx-Frame in Tx-Queue stellen
			if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE)
					ESP_LOGW("Tx-Queue: ", "Scan fail");

			vTaskDelay(50*MS);
			if (rtc_wifi_channel != 0) {
				rtc_no_response_serie = 0;
				break;
			}
		}

		uint32_t sleeptime_ms;
		if (rtc_wifi_channel == 0) {
			//wenn nach einem Durchlauf kein Kanal gefunden wurde -> Gerät in DeepSleep
			sleeptime_ms = 5*1000;
			if (rtc_cnt_no_scan++ > 3) sleeptime_ms = 10*60*1000; //Sleep-Time nach  n Versuchen verlängern
			if (rtc_cnt_no_scan   > 6) sleeptime_ms = 60*60*1000;
		} else {
			ESP_ERROR_CHECK( esp_wifi_set_channel(rtc_wifi_channel, WIFI_SECOND_CHAN_NONE));
printf("Set Channel: %d\n", rtc_wifi_channel);
			sleeptime_ms = 5*1000;	//1. Ruhezeit
		}

		esp_sleep_enable_timer_wakeup(sleeptime_ms * 1000);
		rtc_gpio_isolate(GPIO_NUM_15); //Ruhestrom bei externem Pulldown reduzieren
		esp_deep_sleep_disable_rom_logging();
printf("Sleep for %dms\n", sleeptime_ms);
		esp_deep_sleep_start();

	} else {
		rtc_wifi_channel = ch;
		ESP_ERROR_CHECK( esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE) );
	}
}

void wiog_wifi_sensor_init() {

	esp_sleep_wakeup_cause_t rst_reason = esp_sleep_get_wakeup_cause();
    if ((rst_reason != ESP_SLEEP_WAKEUP_ULP)  &&
    	(rst_reason != ESP_SLEEP_WAKEUP_EXT1) &&
		(rst_reason != ESP_SLEEP_WAKEUP_TIMER)) {
    	//frisch initialisieren
    	rtc_wifi_channel = 0;	//Channel-scan veranlassen
    	rtc_cnt_no_scan = 0;
    	rtc_cycles = 0;
    	rtc_no_response = 0;
    	rtc_no_response_serie = 0;
    	rtc_interval_ms = SENSOR_DEF_SLEEP_TIME_MS;
    	rtc_tx_pwr = MAX_TX_POWER;
    } else {
    	interval_ms = rtc_interval_ms;
    }

    //bei wiederholt fehlendem Response -> ChannelScan erzwingen
    if (rtc_no_response_serie > 1) rtc_wifi_channel = 0;


	//Rx-Queue -> Low_Prio Verarbeitung empfangener Daten
	wiog_rx_queue = xQueueCreate(WIOG_RX_QUEUE_SIZE, sizeof(wiog_event_rxdata_t));
	xTaskCreate(wiog_rx_processing_task, "wiog_rx_task", 4096, NULL, 12, NULL);

	//Tx-Queue - Unterprogramme stellen zu sendende Daten in die Queue
	wiog_tx_queue = xQueueCreate(WIOG_TX_QUEUE_SIZE, sizeof(wiog_event_txdata_t));
	xTaskCreate(wiog_tx_processing_task, "wiog_tx_task", 2048, NULL, 5, NULL);

	//warten auf ACK, Abbruch Tx-Wiederholung
	ack_timeout_Semaphore = xSemaphoreCreateBinary();
	//vor DeepSleep
	goto_sleep_Semaphore = xSemaphoreCreateBinary();

	esp_netif_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_country(&wifi_country_de) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR));
	esp_wifi_set_promiscuous(true);
	esp_wifi_set_promiscuous_rx_cb(&wiog_receive_packet_cb);
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
	ESP_ERROR_CHECK(esp_wifi_start());

	wifi_promiscuous_filter_t filter;
	filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
	ESP_ERROR_CHECK( esp_wifi_set_promiscuous_filter(&filter) );

	ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(rtc_tx_pwr));

	//Kanal setzen oder Channel-Scan
	wiog_set_channel(rtc_wifi_channel);

	my_uid = get_uid();	//Geräte-ID berechnen
}

void wiog_wifi_sensor_goto_sleep(wakeup_src_t wus) {
	//Antwort des Gateway/Node abwarten
	if (xSemaphoreTake(goto_sleep_Semaphore, 500*MS) != pdTRUE) {
		rtc_no_response++;
		rtc_no_response_serie++;
	} else {
		rtc_no_response_serie = 0;
	}

	//Bereichsprüfung interval
	if (interval_ms < SENSOR_MIN_SLEEP_TIME_MS) rtc_interval_ms = SENSOR_MIN_SLEEP_TIME_MS;
	else if (interval_ms > SENSOR_MAX_SLEEP_TIME_MS) rtc_interval_ms = SENSOR_MAX_SLEEP_TIME_MS;
	else rtc_interval_ms = interval_ms;

	//Sendeleistung im nächsten Zyklus
	//Begrenzung auf -3dB / +6dB je Zyklus
	if (tx_pwr_delta_dB < -3) tx_pwr_delta_dB = -3;
	if (tx_pwr_delta_dB >  6) tx_pwr_delta_dB =  6;

	rtc_tx_pwr += tx_pwr_delta_dB * 4;
	if (rtc_tx_pwr > MAX_TX_POWER) rtc_tx_pwr = MAX_TX_POWER;
	if (rtc_tx_pwr < MIN_TX_POWER) rtc_tx_pwr = MIN_TX_POWER;

	//alle Wakeupquellen zurücksetzen
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

	switch (wus) {
	case WS_MAIN:
		ESP_ERROR_CHECK( esp_sleep_enable_timer_wakeup(rtc_interval_ms * 1000)); //ESP starten nach x mx
		break;
	case WS_ULP:
	    ESP_ERROR_CHECK( ulp_set_wakeup_period(0, rtc_interval_ms *1000));  //ulp starten nach x ms
		ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup());
		break;
	} //case

	rtc_gpio_isolate(GPIO_NUM_15); //Ruhestrom bei externem Pulldown reduzieren
    esp_deep_sleep_disable_rom_logging();
	rtc_onTime += esp_timer_get_time() / 1000;
    #ifdef DEBUG_X
    	printf("[%04d]Goto DeepSleep for %dms\n", now(), rtc_interval_ms);
	#endif
	esp_deep_sleep_start();
}
