#include "freertos/FreeRTOS.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "mbedtls/aes.h"
#include "esp32/rom/crc.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#include "nvs_flash.h"
#include "string.h"

#include "../../wiog_include/wiog_system.h"
#include "../../wiog_include/wiog_data.h"
#include "wiog_actor.h"

#undef  LOG_LOCAL_LEVEL	//Warnhinweis vermeiden
#define LOG_LOCAL_LEVEL ESP_LOG_NONE	//s. readme.txt
//zusätzlich Bootloader-msg  mit GPIO_15 -> low unterdrücken

// --------------------------------------------------------------------------------

uint32_t actual_frame_id; //ID (Random) des Tx-Paketes -> f. warten auf Antwort-Frame
uint32_t acked_frame_id;

species_t species = ACTOR;
uint8_t slot = 0;

SemaphoreHandle_t semph_wfa = NULL;	//Wait for ACK

//Daten
uint8_t   wifi_channel;
uint16_t  cnt_no_response;
uint32_t  cycle;
uint16_t  dev_uid; //Geräte-ID wird aus efuse_MAC berechnet

//Standard-Interval für Actoren
uint32_t interval_ms = 60*1000;

int64_t timer;	//TEST !!!!!!!

//WIoG - Wireless Internet of Garden
#define WIOG_RX_QUEUE_SIZE 12
static xQueueHandle wiog_rx_queue;

#define WIOG_TX_QUEUE_SIZE 6
static xQueueHandle wiog_tx_queue;

//Liste, Mehrfach-Repeat verhindern, Richtung zum Gateway
#define HDLA_SZ 8
uint8_t hdlA_ix;
uint32_t hdlA_ids[HDLA_SZ];

// Richtung vom Gateway
#define HDLB_SZ 8
uint8_t hdlB_ix;
uint32_t hdlB_ids[HDLB_SZ];

SemaphoreHandle_t return_timeout_Semaphore = NULL;

//Prototypen
//static void repeat_frame_to_gw_task (void *pvParameter);
bool is_handledA(uint32_t fid);
bool is_handledB(uint32_t fid);

//CallBack - Auswertung GW-Daten in main /Wenn callback registriert wurde
wiog_rx_data_cb_t main_rx_data_cb = NULL;

//---------------------------------------------------------------------------------------------------------------


//Wifi-Rx-Callback im Sniffermode - Daten in die Rx-Queue stellen
IRAM_ATTR  void wiog_receive_packet_cb(void* buff, wifi_promiscuous_pkt_type_t type)
{
	const wifi_promiscuous_pkt_t   *ppkt = (wifi_promiscuous_pkt_t *)buff;
	const wiog_data_frame_t  *ipkt = (wiog_data_frame_t *)ppkt->payload;
	const wiog_header_t *header =  &ipkt->header;

	//nur fehlerfreie Pakete des eigenen Netzes bearbeiten
	if ((ppkt->rx_ctrl.rx_state != 0) || (memcmp(header->mac_net, &mac_net, 6) !=0)) return;

	wiog_event_rxdata_t frame;
	memcpy(&frame.rx_ctrl, &ppkt->rx_ctrl, sizeof(wifi_pkt_rx_ctrl_t));
	memcpy(&frame.wiog_hdr, header, sizeof(wiog_header_t));
	frame.data_len = ppkt->rx_ctrl.sig_len - sizeof(wiog_header_t) - 4; //Datenläne ohne WIOG-Header und  FCS
	frame.data = (uint8_t*)malloc(frame.data_len);
	memcpy(frame.data, &ipkt->data, frame.data_len);

	if (xQueueSend(wiog_rx_queue, &frame, portMAX_DELAY) != pdTRUE) {
			ESP_LOGW("wiog_rx_cb: ", "receive queue fail");
			free(frame.data);
	}
}


//Verarbeitung eines empfangenen Datenpaketes
//kompletter Sniffer-Frame im Event-Parameter der Queue
//Payload muss in Queue-Loop freigegeben werden
IRAM_ATTR static void wiog_rx_processing_task(void *pvParameter)
{
	wiog_event_rxdata_t evt;

	while ((xQueueReceive(wiog_rx_queue, &evt, portMAX_DELAY) == pdTRUE)) {

		wifi_pkt_rx_ctrl_t *pRx_ctrl = &evt.rx_ctrl;
		wiog_header_t *pHdr = &evt.wiog_hdr;

		// Antwort auf Channel-Scan --------------------------------------
		if ((wifi_channel == 0) && (pHdr->vtype == ACK_FOR_CHANNEL))
		{
			wifi_channel = pHdr->channel;	//Arbeitskanal wird in jedem WIOG-Header geliefert
		}

		//Daten vom Gateway entschlüsseln und verarbeiten -------------------
		//als Antwort auf zuvor gesendeten Tx-Frame ID
		if ((dev_uid == pHdr->uid) && (actual_frame_id == pHdr->frameid) && (pHdr->vtype == RETURN_FROM_GW)) {

			actual_frame_id++;	// nur einmal bearbeiten -> id verfälschen

			//Länge des verschlüsselten Datenblocks
			int blocksz = evt.data_len;
			if (blocksz > 0) {
				//CBC-AES-Key
				uint8_t key[] = {AES_KEY};
				// Key um Frame.ID ergänzen
				uint32_t u32 = pHdr->frameid;
				key[28] = (uint8_t)u32;
				key[29] = (uint8_t)(u32>>=8);
				key[30] = (uint8_t)(u32>>=8);
				key[31] = (uint8_t)(u32>>=8);

				uint8_t payload[blocksz];
				cbc_decrypt(evt.data, payload, blocksz, key, sizeof(key));

				//Verarbeitung der Daten
				if (evt.data_len >= sizeof(management_t)) {
					//Management-Header des Gateway
					management_t* pGw_hdr = malloc(sizeof(management_t));
					//Interval des nächsten Betriebszyklus
					memcpy(pGw_hdr, payload, sizeof(management_t));

					//Gültigkeit der GW-Daten prüfen
					if (pGw_hdr->sid == SYSTEM_ID) {

						//Interval bis zum nächsten Betriebszyklus
						uint32_t iv = pGw_hdr->interval;
						if ((iv >= MIN_ACTOR_INTERVAL_MS ) && (iv <= MAX_ACTOR_INTERVAL_MS))
							interval_ms = iv;
						else
							interval_ms = DEF_ACTOR_INTERVAL_MS;

						//falls unterschiedliche Kanäle in Wiog-hdr und GW-hdr -> Channel-Scan (in Main-Loop veranlassen)
						if (pHdr->channel != pGw_hdr->wifi_channel)	wifi_channel = 0;

						//weitere Datenauswertung in main
						if (main_rx_data_cb != NULL) {
							main_rx_data_cb(payload);
						}

						//Empfang wurde bestätigt -> Tx- Widerholung abbrechen
						xSemaphoreGive(return_timeout_Semaphore);

					}
					free(pGw_hdr);
				}
			}
		}	// return from gw

		//Steuerbefehl des GW an Actor
		if ((dev_uid == pHdr->uid) && (pHdr->vtype == DATA_TO_ACTOR)) {
			//Länge des verschlüsselten Datenblocks
			int blocksz = evt.data_len;
			if (blocksz > 0) {
				//CBC-AES-Key
				uint8_t key[] = {AES_KEY};
				// Key um Frame.ID ergänzen
				uint32_t u32 = pHdr->frameid;
				key[28] = (uint8_t)u32;
				key[29] = (uint8_t)(u32>>=8);
				key[30] = (uint8_t)(u32>>=8);
				key[31] = (uint8_t)(u32>>=8);

				uint8_t payload[blocksz];
				cbc_decrypt(evt.data, payload, blocksz, key, sizeof(key));

				//Verarbeitung der Daten
				//weitere Datenauswertung in main
				if (main_rx_data_cb != NULL) {
					main_rx_data_cb(payload);
				}
			}
		}
		// ================================================================
		// Repeater-Funktion -> Datenpaket von Device an Gateway weiterleiten
		if ((pHdr->vtype == DATA_TO_GW) && (species == REPEATER) && (!is_handledA(pHdr->frameid)) ) {

			//in Liste eintragen, um Wiederholungen zu vermeiden
			hdlA_ids[hdlA_ix] = pHdr->frameid;
			hdlA_ix++;
			hdlA_ix &= HDLA_SZ-1;	//Ringpuffer

			uint8_t buf[pRx_ctrl->sig_len - 4];
			memcpy(&buf[0], pHdr, sizeof(wiog_header_t));
			memcpy(&buf[sizeof(wiog_header_t)], evt.data, evt.data_len);

			wiog_event_txdata_t tx_frame;
			tx_frame.wiog_hdr = evt.wiog_hdr;	//Header kopieren
			tx_frame.crypt_data = false;		//Daten sind bereits verschlüsselt
			tx_frame.data_len = evt.data_len;
			tx_frame.target_time = esp_timer_get_time() + slot * 3000;	//Weiterleitung an GW im zugewiesenen Timesslot

			//Rx-Daten des ersten Repeaters
			if (pHdr->mac_from[5] < REPEATER) {
				tx_frame.wiog_hdr.tagA = pRx_ctrl->rssi - pRx_ctrl->noise_floor; //SNR des Dev am Repeater
				tx_frame.wiog_hdr.tagB = tx_frame.wiog_hdr.mac_from[5]; 		 //Absender-MAC des Dev
				tx_frame.wiog_hdr.tagC = species + slot;						 //MAC des 1. Repeaters
			}

			tx_frame.wiog_hdr.mac_from[5] = species + slot;			//MAC des Rep als Absender
			tx_frame.data = (uint8_t*)malloc(tx_frame.data_len);	//Payload 1:1 weiterleiten
			memcpy(tx_frame.data, evt.data, evt.data_len);

			if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE) {
				ESP_LOGW("Tx: ", "Tx-queue fail - 001");
				free(tx_frame.data);
			}
//printf("RepToGW:  0x%08x\n", tx_frame.wiog_hdr.frameid);
		}

		// Repeater-Funktion -> Datenpaket von Gateway Device weiterleiten, wen es:
		//  - kein Actor-Paket an die eigene UID ist
		//  - Actor als Repeater fungiert
		//  - Das Datenpaket in dieser Richtugn noch nicht empfangen wurde
		if ((pHdr->vtype == RETURN_FROM_GW) && (pHdr->uid != dev_uid) &&  (species == REPEATER) && (!is_handledB(pHdr->frameid)) ) {

			//in Liste eintragen, um Wiederholungen zu vermeiden
			hdlB_ids[hdlB_ix] = pHdr->frameid;
			hdlB_ix++;
			hdlB_ix &= HDLB_SZ-1;	//Ringpuffer

			pHdr->mac_from[5] = species + slot;
			uint8_t buf[pRx_ctrl->sig_len - 4];
			memcpy(&buf[0], pHdr, sizeof(wiog_header_t));
			memcpy(&buf[sizeof(wiog_header_t)], evt.data, evt.data_len);

			wiog_event_txdata_t tx_frame;
			tx_frame.wiog_hdr = evt.wiog_hdr;	//Header kopieren
			tx_frame.crypt_data = false;		//Daten sind bereits verschlüsselt
			tx_frame.data_len = evt.data_len;
			tx_frame.target_time = esp_timer_get_time() + slot * 3000;

			//Header anpassen
			tx_frame.wiog_hdr.mac_from[5] = species + slot;			//Absender anpassen
			tx_frame.data = (uint8_t*)malloc(tx_frame.data_len);
			memcpy(tx_frame.data, evt.data, evt.data_len);

			if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE) {
				ESP_LOGW("Tx: ", "Tx-queue fail - 002");
				free(tx_frame.data);
			}
//printf("RepToDev: 0x%08x\n", tx_frame.wiog_hdr.frameid);
		}

		free(evt.data);
	}	//while
}


//Senden eines Datenpaketes
IRAM_ATTR void wiog_tx_processing_task(void *pvParameter) {

	wiog_event_txdata_t evt;

	while ((xQueueReceive(wiog_tx_queue, &evt, portMAX_DELAY) == pdTRUE)) {

		uint16_t tx_len = get_blocksize(evt.data_len, AES_KEY_SZ) + sizeof(wiog_header_t);
		uint8_t buf[tx_len];
		bzero(buf, tx_len);

		wifi_second_chan_t ch2;
		esp_wifi_get_channel(&evt.wiog_hdr.channel, &ch2);

		//Header in Puffer kopieren
		memcpy(buf, &evt.wiog_hdr, sizeof(wiog_header_t));

		if (evt.crypt_data) {
			//Datenblock verschlüsseln
			//CBC-AES-Key
			uint8_t key[] = {AES_KEY};
			// Frame-ID => 4 letzten Byres im Key
			uint32_t u32 = evt.wiog_hdr.frameid;
			key[28] = (uint8_t)u32;
			key[29] = (uint8_t)(u32>>=8);
			key[30] = (uint8_t)(u32>>=8);
			key[31] = (uint8_t)(u32>>=8);

			cbc_encrypt(evt.data, &buf[sizeof(wiog_header_t)], evt.data_len, key, sizeof(key));
		} else {
			memcpy(&buf[sizeof(wiog_header_t)], evt.data, evt.data_len);
		}

		//Sendug ggf verzögern
		int64_t del_us = evt.target_time - esp_timer_get_time();
		if (del_us > 0)	ets_delay_us((uint32_t) del_us);
		esp_wifi_80211_tx(WIFI_IF_STA, &buf, tx_len, false);

//if (del_us > 0) printf("Tx-Delay: %dµs\n", (int)del_us);

free(evt.data);
	}
}



//Datenframe managed an Gateway senden
bool send_data_frame(uint8_t* buf) {

	uint8_t len = buf[0];		//Längenbyte
	uint8_t *data = &buf[1];	//Datenbereich

	wiog_event_txdata_t tx_frame;
	tx_frame.wiog_hdr = wiog_get_dummy_header(GATEWAY, species);
	tx_frame.wiog_hdr.uid = dev_uid;
	tx_frame.wiog_hdr.species = species;
	tx_frame.wiog_hdr.vtype = DATA_TO_GW;

	//Wiederholschleife
	for (int i = 0; i < 6; i++) {
		tx_frame.wiog_hdr.frameid = esp_random();
		actual_frame_id = tx_frame.wiog_hdr.frameid;

		tx_frame.data = malloc(len);
		memcpy(tx_frame.data, data, len);
		tx_frame.data_len = len;
		tx_frame.crypt_data = true;
		tx_frame.target_time = 0;

		if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE) {
			free(tx_frame.data);
			ESP_LOGW("Tx-Queue: ", "Tx Data fail");
		}
timer = esp_timer_get_time();
		tx_frame.wiog_hdr.seq_ctrl++;

		//Antwort-Frame abwarten
		if (xSemaphoreTake(return_timeout_Semaphore, 80*MS) == pdTRUE)
			break;

	}	//for Tx-Versuche

	//Frame-ID wurde bei erfolgreichem Response verfälscht
	bool res = actual_frame_id != tx_frame.wiog_hdr.frameid;
	if (!res) cnt_no_response++;
	return res;
}


// ---------------------------------------------------------------------------------

void wiog_actor_init(void) {
	nvs_flash_init();

    //Hard-Reset
    cnt_no_response = 0;
    wifi_channel = 0;

    species = REPEATER;				//testweise als Repeater initialisieren
    interval_ms = DEF_ACTOR_INTERVAL_MS;

	//Rx-Queue -> Verarbeitung empfangener Daten
	wiog_rx_queue = xQueueCreate(WIOG_RX_QUEUE_SIZE, sizeof(wiog_event_rxdata_t));
	xTaskCreate(wiog_rx_processing_task, "wiog_rx_task", 2048, NULL, 12, NULL);

	//Tx-Queue - Unterprogramme stellen zu sendende Daten in die Queue
	wiog_tx_queue = xQueueCreate(WIOG_TX_QUEUE_SIZE, sizeof(wiog_event_txdata_t));
	xTaskCreate(wiog_tx_processing_task, "wiog_tx_task", 2048, NULL, 5, NULL);

	return_timeout_Semaphore = xSemaphoreCreateBinary();

    esp_netif_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_country(&wifi_country_de) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR));
	esp_wifi_set_promiscuous(true);
	esp_wifi_set_promiscuous_rx_cb(&wiog_receive_packet_cb);

	ESP_ERROR_CHECK(esp_wifi_start());


	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

	wifi_promiscuous_filter_t filter;
	filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
	ESP_ERROR_CHECK( esp_wifi_set_promiscuous_filter(&filter) );

	dev_uid = get_uid();	//Geräte-ID berechnen

	//Wifi-Kanal-Scan
	wiog_set_channel(wifi_channel);
	//actor sendet immer mit voller Leistung
    esp_wifi_set_max_tx_power(MAX_TX_POWER);

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
			.data = NULL,
			.target_time = 0	//sofort senden
		};
		tx_frame.wiog_hdr = wiog_get_dummy_header(GATEWAY, species);
		tx_frame.wiog_hdr.vtype = SCAN_FOR_CHANNEL;
		tx_frame.wiog_hdr.uid = dev_uid;
		tx_frame.wiog_hdr.species = species;

		for (ch = 1; ch <= wifi_country_de.nchan; ch++) {
			ESP_ERROR_CHECK( esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE));
			//Tx-Frame in Tx-Queue stellen
			if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE)
					ESP_LOGW("Tx-Queue: ", "Scan fail");
			vTaskDelay(30*MS);
			if (wifi_channel != 0) {
				cnt_no_response = 0;
				break;
			}
		}

		//wenn nach einem Durchlauf kein Kanal gefunden wurde -> Gerät in DeepSleep
		if (wifi_channel == 0) {
			uint32_t sleeptime_ms = 5*1000;
			esp_sleep_enable_timer_wakeup(sleeptime_ms * 1000);
		    rtc_gpio_isolate(GPIO_NUM_15); //Ruhestrom bei externem Pulldown reduzieren
		    esp_deep_sleep_disable_rom_logging();
		    printf("No Wifi-Channel - goto DeepSleep for %dms\n", sleeptime_ms);
			esp_deep_sleep_start();

		} else {
			ESP_ERROR_CHECK( esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE));
		}

	} else {
		wifi_channel = ch;
		ESP_ERROR_CHECK( esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE) );
	}
	printf("Set Wifi-channel: %d\n", wifi_channel);
}


// -----------------------------------------------------------------------------------

//Frame-ID vor kurzem Repeated ??
//Device -> GW
bool is_handledA(uint32_t fid){
	bool res = false;
	for (int i = 0; i < HDLA_SZ; i++) {
		if (hdlA_ids[i] == fid) {
			res = true;
			break;
		}
	}
	return res;
}

//GW -> Device
bool is_handledB(uint32_t fid){
	bool res = false;
	for (int i = 0; i < HDLB_SZ; i++) {
		if (hdlB_ids[i] == fid) {
			res = true;
			break;
		}
	}
	return res;
}

//-----------------------------------------------------------------------------------------------

//Callback in main registrieren
void wiog_rx_register_cb(wiog_rx_data_cb_t pRx_data){
	main_rx_data_cb = pRx_data;
};
