/*
 * wiog_actor.c
 *
 *  Created on: 18.06.2021
 *      Author: joerg
 */

#include "esp_system.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "string.h"
#include "esp_log.h"
#include "esp_private/wifi.h"

#include "wiog_system.h"
#include "wiog_data.h"

#include "wiog_wifi_actor.h"


#define WIOG_RX_QUEUE_SIZE 12
static xQueueHandle wiog_rx_queue;

#define WIOG_TX_QUEUE_SIZE 6
static xQueueHandle wiog_tx_queue;

//Daten
uint8_t   wifi_channel;
uint8_t	  chx;
uint32_t  cnt_no_response;
uint32_t  cnt_no_response_serie;
uint32_t  cycle;
dev_uid_t my_uid; //Geräte-ID wird aus efuse_MAC berechnet

uint32_t interval_ms = 60*1000;		//Standard-Interval für Actoren



SemaphoreHandle_t ack_timeout_Semaphore = NULL;

uint32_t ack_id = 0;		//Vergleich mit FrameID	- Tx-Wiederholungen
uint32_t acked_fid;			//Node-Wiederholung
uint32_t tx_fid;			//aktuelle ID der letzten Sendung

//Prototypen
void set_dbls_fid(uint32_t fid);
bool is_dbls_fid(uint32_t fid);

//-----------------------------------------------------------------------------------------

//Wifi-Rx-Callback im Sniffermode - Daten in die Rx-Queue stellen
//max Paketlänge im promiscuous-Mode = 112Bytes (ESP8266)
IRAM_ATTR  void wiog_receive_packet_cb(void* buff, wifi_promiscuous_pkt_type_t type) {
	const wifi_promiscuous_pkt_t   *ppkt = (wifi_promiscuous_pkt_t *)buff;
	const wiog_data_frame_t  *ipkt = (wiog_data_frame_t *)ppkt->payload;
	const wiog_header_t *header =  &ipkt->header;
	uint32_t len = ppkt->rx_ctrl.sig_mode ? ppkt->rx_ctrl.HT_length : ppkt->rx_ctrl.legacy_length;

	//nur Pakete an eigene UID bearbeiten
	if ((memcmp(header->mac_to, &mac_actor, 6) !=0)	|| (header->uid != my_uid)) return;
	//max Datenlänge (incl Header) im Sniffermode ESP8266
	if (len > 112) return;	//max Paketlänge d. 8266 überschritten

	wiog_event_rxdata_t frame;
	memcpy(&frame.rx_ctrl, &ppkt->rx_ctrl, sizeof(wifi_pkt_rx_ctrl_t));
	memcpy(&frame.wiog_hdr, header, sizeof(wiog_header_t));

	frame.data_len = len - sizeof(wiog_header_t) - 4 +16;
	frame.pdata = (uint8_t*)malloc(frame.data_len);
	memcpy(frame.pdata, &ipkt->data, frame.data_len);
//	frame.timestamp = esp_timer_get_time();	//Empfangszeitpunkt (sinnhaft ??)

	if (xQueueSend(wiog_rx_queue, &frame, portMAX_DELAY) != pdTRUE) {
			ESP_LOGW("wiog_rx_cb: ", "receive queue fail");
			free(frame.pdata);
	}
}


//Verarbeitung eines empfangenen Datenpaketes
//kompletter Sniffer-Frame im Event-Parameter der Queue
//Payload muss in Queue-Loop freigegeben werden
IRAM_ATTR static void wiog_rx_processing_task(void *pvParameter)
{
	wiog_event_rxdata_t evt;

	while ((xQueueReceive(wiog_rx_queue, &evt, portMAX_DELAY) == pdTRUE)) {
//		wifi_pkt_rx_ctrl_t *pRx_ctrl = &evt.rx_ctrl;

		wiog_header_t *pHdr = &evt.wiog_hdr;
/*
		// Antwort auf eigenen Channel-Scan ---------------------------------
		// Auswertung der eigenen UID nicht erforderlich
		if (wifi_channel == 0) {
			if (pHdr->vtype == ACK_FOR_CHANNEL) {
				wifi_channel = pHdr->channel;	//Arbeitskanal wird in jedem WIOG-Header geliefert
			}
		}
		else
*/
		//Frame an eigene UID adressiert
		if (pHdr->uid == my_uid ) {
			wifi_channel = pHdr->channel;	//Arbeitskanal wird in jedem WIOG-Header geliefert
			chx = pHdr->channel;			//Channel-Scan abbrechen

			// ACK des GW auf aktuelle Frame-ID, Tx-Wiederholungen stoppen
			if ((pHdr->vtype == ACK_FROM_GW) && (pHdr->frameid == tx_fid)) {
				//Tx-Wiederholungen stoppen
				tx_fid = 0;
				xSemaphoreGive(ack_timeout_Semaphore);

				//Interval-Info auf Plausibilität prüfen
				if ((pHdr->interval_ms >= ACTOR_MIN_SLEEP_TIME_MS) && (pHdr->interval_ms <= ACTOR_MAX_SLEEP_TIME_MS))
					interval_ms = pHdr->interval_ms;

				(*cb_rx_ack_handler)(pHdr);	//Callback actor_main
			}

			//Empfang eines Datenframes vom RPi-Gateway
			if (pHdr->vtype == DATA_FROM_GW) {
				//Ack an Gateway senden
				wiog_event_txdata_t* ptx_frame = malloc(sizeof(wiog_event_txdata_t));
				ptx_frame->crypt_data = false,
				ptx_frame->target_time = 0,
				ptx_frame->data_len = 0,
				ptx_frame->pdata = NULL,
				//Header modifiziert als ACK zurücksenden
				ptx_frame->wiog_hdr = evt.wiog_hdr;
				ptx_frame->wiog_hdr.mac_from[5] = ACTOR;
				ptx_frame->wiog_hdr.mac_to[5] = GATEWAY;
				ptx_frame->wiog_hdr.vtype = ACK_TO_GW;
				ptx_frame->tx_max_repeat = 0;	//keine Wiederholung + kein ACK erwartet
//				ptx_frame->h_timer = NULL;
				//eigene Sendeleistungs-Ix an GW liefern
				esp_wifi_get_max_tx_power(&ptx_frame->wiog_hdr.txpwr);

				if (xQueueSend(wiog_tx_queue, ptx_frame, portMAX_DELAY) != pdTRUE)
					ESP_LOGW("Tx-Queue: ", "Channelscan fail");

				free(ptx_frame);

				//an Datenauswertung übergeben
				if (!is_dbls_fid(evt.wiog_hdr.frameid)) { //nur wenn Frame-ID noch nicht behandelt wurde
					set_dbls_fid(evt.wiog_hdr.frameid);	//FID in Liste eintragen

					//Datenblock entschlüsseln
					uint8_t buf[evt.data_len]; //decrypt Data nicht größer als encrypted Data
					payload_t* ppl = (payload_t*)buf;
					if ((wiog_decrypt_data(evt.pdata, buf, evt.data_len, evt.wiog_hdr.frameid) == 0) &&
						//integrität des Datenblocks prüfen
						(ppl->man.len <= MAX_DATA_LEN) &&
						(ppl->man.crc16 == crc16((uint8_t*)&ppl->man.len, ppl->man.len-2)))
							//Callback actor_main
							(*cb_rx_data_handler)(pHdr, (payload_t*)buf, ppl->man.len);
					else printf("Payload-Error\n");
				}
			}
		}
		free(evt.pdata);
	}	//while rx queue
}


//Zentraler Task zum Senden jedes Frame-Typs
IRAM_ATTR void wiog_tx_processing_task(void *pvParameter) {

	wiog_event_txdata_t evt;

	while ((xQueueReceive(wiog_tx_queue, &evt, portMAX_DELAY) == pdTRUE)) {
		uint16_t tx_len = get_blocksize(evt.data_len) + sizeof(wiog_header_t);
		uint8_t buf[tx_len];
		bzero(buf, tx_len);

		//ggf Kanalkorrektur
		uint8_t ch;
		wifi_second_chan_t ch2;
		ESP_ERROR_CHECK( esp_wifi_get_channel(&ch, &ch2) );
		if (ch != wifi_channel) {
			ESP_ERROR_CHECK( esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE));
			printf("Change Wifi-channel: %d -> %d\n", ch, wifi_channel);
		};
		evt.wiog_hdr.channel =   wifi_channel;

		//Header in Puffer kopieren
		memcpy(buf, &evt.wiog_hdr, sizeof(wiog_header_t));

		if (evt.crypt_data) {
			//Datenblock verschlüsseln
			wiog_encrypt_data(evt.pdata, &buf[sizeof(wiog_header_t)], evt.data_len, evt.wiog_hdr.frameid);
		} else {
			memcpy(&buf[sizeof(wiog_header_t)], evt.pdata, evt.data_len);
		}

		((wiog_header_t*) buf)->seq_ctrl = 0;
		//max Wiederholungen bis ACK von GW oder Node
		tx_fid = evt.wiog_hdr.frameid;
		for (int i = 0; i <= evt.tx_max_repeat; i++) {
			//Frame senden
			//!!! LSB im ersten MAC-Byte muss "1" sein, sonst unkontrollierte Tx-Wiederholungen (Bug in RTOS-IDF (ESP32))
			ESP_ERROR_CHECK( esp_wifi_80211_tx(WIFI_IF_STA, buf, tx_len, false));

			if (evt.tx_max_repeat == 0) {
				vTaskDelay(50*MS);
				break;	//1x Tx ohne ACK
			}
			//warten auf Empfang eines ACK
			if (xSemaphoreTake(ack_timeout_Semaphore, TX_REPEAT_INTERVAL) == pdTRUE) {
				cnt_no_response_serie = 0;
				vTaskDelay(50*MS);
				break; //ACK empfangen -> Wiederholung abbrechen
			}

			vTaskDelay(10*MS);
			//bei fehlendem ACK
			((wiog_header_t*) buf)->seq_ctrl++ ;	//Sequence++

			if (i == evt.tx_max_repeat) {
				cnt_no_response++;
				cnt_no_response_serie++;
			}
		}
		free(evt.pdata);
	}
}

// Datenverarbeitung ----------------------------------------------------------------------------

//Eintragen der Management-Daten in den Payload
//Aufrufer aktualisiert später die Anzahl der Datensätze
void set_management_data (management_t* pMan) {
	pMan->crc16 = 0;
	pMan->len = 0;
	pMan->uid = my_uid;
	pMan->wifi_channel = wifi_channel;
	pMan->species = ACTOR;
	pMan->version = version;
	pMan->revision = revision;
//	pMan->cycle = cycle++;
	pMan->cnt_no_response = cnt_no_response;
	ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&pMan->tx_pwr));
	//freier Heap (Test Mem-Leaks)
	pMan->sz_heap = esp_get_free_heap_size();
	pMan->cnt_entries = 0;
}

//Datenframe managed an Gateway senden
void send_data_frame(payload_t* buf, uint16_t len, species_t spec) {

	uint8_t *pdata = (uint8_t*)buf;	//Datenbereich

	wiog_event_txdata_t tx_frame;
	tx_frame.wiog_hdr = wiog_get_dummy_header(GATEWAY, spec);
	tx_frame.wiog_hdr.uid = my_uid;
	tx_frame.wiog_hdr.species = spec;
	tx_frame.wiog_hdr.vtype = DATA_TO_GW;
	tx_frame.wiog_hdr.frameid = esp_random();
	tx_frame.tx_max_repeat = TX_REPEAT_CNT_MAX;		//max Wiederholungen, ACK erwartet

	tx_frame.pdata = malloc(len);
	memcpy(tx_frame.pdata, pdata, len);
	tx_frame.data_len = len;
	tx_frame.crypt_data = true;
	tx_frame.target_time = 0;

	if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE)
		ESP_LOGW("Tx-Queue: ", "Tx Data fail");
}


// Channel - Scan -----------------------------------------------------------------------------
//Kanal wird in jedem Tx-Process geprüft und ggf. gesetzt
void wiog_set_channel(uint8_t ch) {
	if (ch == 0) {
		wiog_event_txdata_t tx_frame = {
			.crypt_data = false,
			.data_len = 0,
			.pdata = NULL,
			.target_time = 0
		};
		tx_frame.wiog_hdr = wiog_get_dummy_header(GATEWAY, ACTOR);
		tx_frame.wiog_hdr.vtype = SCAN_FOR_CHANNEL;
		tx_frame.wiog_hdr.uid = my_uid;
		tx_frame.wiog_hdr.species = ACTOR;	//Scan als Actor
		tx_frame.wiog_hdr.frameid = 0;

		chx = 0; //wird in rx ack gesetzt
		for (ch = 1; ch <= wifi_country_de.nchan; ch++) {
			wifi_channel = ch;	//Kanal in Tx-Queue einstellen
			//Tx-Frame in Tx-Queue stellen
			if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE)
					ESP_LOGW("Tx-Queue: ", "Scan fail");
			vTaskDelay(150*MS);
			//wifi_channel wird bei Empfang Ack gesetzt
			if (chx != 0) {
				cnt_no_response_serie = 0;
				break;
			}
		}

		//wenn nach einem Durchlauf kein Kanal gefunden wurde -> 5 Sek Pause
		if (chx == 0) {
			wifi_channel = 0;
			vTaskDelay(5000*MS);
		}
	} else {
		wifi_channel = ch;
	}
	ESP_ERROR_CHECK( esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE));
	printf("Set Wifi-channel: %d\n", wifi_channel);
}


void wiog_wifi_actor_init() {
    cnt_no_response = 0;
    cnt_no_response_serie = 0;
    wifi_channel = 0;

    interval_ms = ACTOR_DEF_SLEEP_TIME_MS;	//wird in ACK-Frame zugewiesen

	//Rx-Queue -> Verarbeitung empfangener Daten
	wiog_rx_queue = xQueueCreate(WIOG_RX_QUEUE_SIZE, sizeof(wiog_event_rxdata_t));
	xTaskCreate(wiog_rx_processing_task, "wiog_rx_task", 4096, NULL, 8, NULL);

	//Tx-Queue - Unterprogramme stellen zu sendende Daten in die Queue
	wiog_tx_queue = xQueueCreate(WIOG_TX_QUEUE_SIZE, sizeof(wiog_event_txdata_t));
	xTaskCreate(wiog_tx_processing_task, "wiog_tx_task", 4096, NULL, 9, NULL);

	ack_timeout_Semaphore = xSemaphoreCreateBinary();

	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_country(&wifi_country_de) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B));
    ESP_ERROR_CHECK(esp_wifi_start());

	wifi_promiscuous_filter_t filter;
	filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;

    ESP_ERROR_CHECK( esp_wifi_set_promiscuous_rx_cb(&wiog_receive_packet_cb));
    ESP_ERROR_CHECK( esp_wifi_set_promiscuous_filter(&filter) );
	esp_wifi_set_promiscuous(true);

	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

	my_uid = get_uid();	//Geräte-ID berechnen

	//Wifi-Kanal-Scan
	wiog_set_channel(0);
    cnt_no_response = 0;
    cnt_no_response_serie = 0;

	//actor sendet immer mit voller Leistung
	ESP_ERROR_CHECK( esp_wifi_set_max_tx_power(MAX_TX_POWER));
}


// Hilfsfunktionen --------------------------------------------------------------------------

//Liste der letzten empfangenen Frame-ID
//doppelte Bearbeitung verhindern
//Ringpuffer
#define DBLS_FID_SIZE 4	// 2^n !!
uint32_t dbls_fid[DBLS_FID_SIZE];
uint8_t  ix_dbls_fid = 0;


void set_dbls_fid(uint32_t fid){
	dbls_fid[ix_dbls_fid] = fid;
	ix_dbls_fid++;
	ix_dbls_fid &= DBLS_FID_SIZE - 1;	//Ringpuffer
}

bool is_dbls_fid(uint32_t fid) {
	bool res = false;
	for (int i=0; i<DBLS_FID_SIZE; i++) {
		if (dbls_fid[i] == fid) {
			res = true;
			break;
		}
	}
	return res;
}

