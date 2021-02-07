#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "string.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp_log.h"

#include "../../wiog_include/wiog_system.h"
#include "../../wiog_include/wiog_data.h"

#define SENSOR_MIN_SLEEP_TIME_MS  10*SEK
#define SENSOR_DEF_SLEEP_TIME_MS  10*MIN
#define SENSOR_MAX_SLEEP_TIME_MS  24*STD


#undef  LOG_LOCAL_LEVEL	//Warnhinweis vermeiden
#define LOG_LOCAL_LEVEL ESP_LOG_NONE	//s. readme.txt
//zusätzlich Bootloader-msg  mit GPIO_15 -> low unterdrücken


//öffentliche Variablen
RTC_DATA_ATTR static uint32_t rtc_onTime;
RTC_DATA_ATTR static uint8_t  rtc_no_response_cnt;

//Daten, die einen Deep-Sleep überstehen müssen
RTC_DATA_ATTR static uint8_t  rtc_wifi_channel;
RTC_DATA_ATTR static uint32_t rtc_cycles;
RTC_DATA_ATTR static uint32_t rtc_interval_ms;
RTC_DATA_ATTR static int8_t   rtc_tx_pwr;		//Steuerung Sendeleistung
RTC_DATA_ATTR static uint32_t rtc_cnt_no_scan;  //Fehlversuche Channelscan

// --------------------------------------------------------------------------------
uint16_t dev_uid; //Geräte-IS wird aus efuse_MAC berechnet
uint32_t actual_frame_id; //ID (Random) des Tx-Paketes -> f. warten auf Antwort-Frame
uint32_t acked_frame_id;

species_t species = SENSOR;
bool waked_up;

SemaphoreHandle_t semph_wfa = NULL;	//Wait for ACK


int64_t timer;	//TEST !!!!!!!

//SystemVariablen zur Steurung der Sensoren
#define MAX_SYSVAR  8 //Systemvariablen [0..7], Data-Entries im Quittungspaket
RTC_DATA_ATTR static int32_t   rtc_sysvar[MAX_SYSVAR] __attribute__((unused)); 	//0..7

//WIoG - Wireless Internet of Garden
#define WIOG_RX_QUEUE_SIZE 12
static xQueueHandle wiog_rx_queue;

#define WIOG_TX_QUEUE_SIZE 6
static xQueueHandle wiog_tx_queue;

SemaphoreHandle_t return_timeout_Semaphore = NULL;

//---------------------------------------------------------------------------------------------------------------

//Wifi-Rx-Callback im Sniffermode - Daten in die Rx-Queue stellen
IRAM_ATTR void wiog_receive_packet_cb(void* buff, wifi_promiscuous_pkt_type_t type)
{
	const wifi_promiscuous_pkt_t   *ppkt = (wifi_promiscuous_pkt_t *)buff;
	const wiog_data_frame_t  *ipkt = (wiog_data_frame_t *)ppkt->payload;
	const wiog_header_t *pHdr =  &ipkt->header;

	//nur fehlerfreie Pakete des eigenen Netzes bearbeiten
	if ((ppkt->rx_ctrl.rx_state != 0) || (memcmp(pHdr->mac_net, &mac_net, sizeof(mac_addr_t)) !=0)) return;

	//nur per UID adressierte Pakete akzeptieren
	if (pHdr->uid != dev_uid) return;
	if (pHdr->vtype == DATA_TO_GW) return; //keine Paket-Wiederholungen auswerten

	wiog_event_rxdata_t frame;
	memcpy(&frame.rx_ctrl, &ppkt->rx_ctrl, sizeof(wifi_pkt_rx_ctrl_t));
	memcpy(&frame.wiog_hdr, pHdr, sizeof(wiog_header_t));
	frame.data_len = ppkt->rx_ctrl.sig_len - sizeof(wiog_header_t) - 4; //ohne FCS
	frame.data = (uint8_t*)malloc(frame.data_len);
	memcpy(frame.data, &ipkt->data, frame.data_len);

	//Event in die Rx-Queue stellen
	if (xQueueSend(wiog_rx_queue, &frame, portMAX_DELAY) != pdTRUE) {
			ESP_LOGW("Rx_Quue", "receive queue fail");
			free(frame.data);
	}
}


//Verarbeitung eines empfangenen Datenpaketes
IRAM_ATTR static void wiog_rx_processing_task(void *pvParameter)
{
	wiog_event_rxdata_t evt;

	while ((xQueueReceive(wiog_rx_queue, &evt, portMAX_DELAY) == pdTRUE)) {

		wifi_pkt_rx_ctrl_t *pRx_ctrl = &evt.rx_ctrl;
		wiog_header_t *pHdr = &evt.wiog_hdr;
int tix = (esp_timer_get_time() - timer) / 1000;
printf("from: %02x | 0x%08x | SNR: %02d | T: %dms\n",
	pHdr->mac_from[5], pHdr->frameid, pRx_ctrl->rssi - pRx_ctrl->noise_floor, tix);

		// Antwort auf Channel-Scan ------------------------------------------
		if ((rtc_wifi_channel == 0) && (pHdr->vtype == ACK_FOR_CHANNEL))
		{
			rtc_wifi_channel = pHdr->channel;	//Arbeitskanal wird in jedem WIOG-Header geliefert
		}

		//Daten vom Gateway entschlüsseln und verarbeiten -------------------
		//als Antwort auf zuvor gesendeten Tx-Frame ID
		if ((actual_frame_id == pHdr->frameid) && (pHdr->vtype == RETURN_FROM_GW)) {

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

				//vom GW empfohlener Korrekturwert zur Einpegelung auf IDEAL_SNR
				if (pHdr->txpwr < -4) rtc_tx_pwr += -4; else rtc_tx_pwr += pHdr->txpwr;
				if (rtc_tx_pwr > MAX_TX_POWER) rtc_tx_pwr = MAX_TX_POWER;
				if (rtc_tx_pwr < MIN_TX_POWER) rtc_tx_pwr = MIN_TX_POWER;

				//Tx- Wiederholung abbrechen
				xSemaphoreGive(return_timeout_Semaphore);
			}

			free(evt.data);
		}	// Data-Frame
	}	// Rx-Queue
}


//Senden eines Datenpaketes
IRAM_ATTR void wiog_tx_processing_task(void *pvParameter) {

	wiog_event_txdata_t evt;

	while ((xQueueReceive(wiog_tx_queue, &evt, portMAX_DELAY) == pdTRUE)) {

		uint16_t tx_len = get_blocksize(evt.data_len, AES_KEY_SZ) + sizeof(wiog_header_t);
		uint8_t buf[tx_len];
		bzero(buf, tx_len);

		//Kanal und Tx-Power-Index im Header übertragen
		wifi_second_chan_t ch2;
		esp_wifi_get_channel(&evt.wiog_hdr.channel, &ch2);
		esp_wifi_get_max_tx_power(&evt.wiog_hdr.txpwr);

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

		esp_wifi_80211_tx(WIFI_IF_STA, &buf, tx_len, false);

#ifdef DEBUG_Xx
		printf("Tx: 0x%08x | %d Bytes | Typ: %02d | %04d =>\n",
				evt.wiog_hdr.frameid, tx_len, evt.wiog_hdr.vtype, evt.wiog_hdr.seq_ctrl);

		int8_t maxpwr;
		esp_wifi_get_max_tx_power(&maxpwr);
		printf("Set Tx-Power: %.2f dBm, %d %d\n", maxpwr *0.25, rtc_tx_pwr, maxpwr);
#endif

		free(evt.data);
	}
}




// Öffentliche Funktionen ================================================================================

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

		tx_frame.wiog_hdr.seq_ctrl++;

		//Antwort-Frame abwarten
		if (xSemaphoreTake(return_timeout_Semaphore, 80*MS) == pdTRUE)
			break;

	}	//for Tx-Versuche

	//Frame-ID wurde bei erfolgreichem Response verfälscht
	bool res = actual_frame_id != tx_frame.wiog_hdr.frameid;
	if (!res) rtc_no_response_cnt++;
	return res;
}

// ---------------------------------------------------------------------------------




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

		rtc_tx_pwr = MAX_TX_POWER;
		esp_wifi_set_max_tx_power(rtc_tx_pwr);

		for (ch = 1; ch <= wifi_country_de.nchan; ch++) {
			ESP_ERROR_CHECK( esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE));
			//Tx-Frame in Tx-Queue stellen
			if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE)
					ESP_LOGW("Tx-Queue: ", "Scan fail");
#ifdef DEBUG_X
    printf("Scan Channel: %d\n", ch);
#endif
			vTaskDelay(30*MS);
			if (rtc_wifi_channel != 0) {
				rtc_no_response_cnt = 0;
				break;
			}
		}

		//wenn nach einem Durchlauf kein Kanal gefunden wurde -> Gerät in DeepSleep
		if (rtc_wifi_channel == 0) {

			uint32_t sleeptime_ms = 5*1000;
			if (rtc_cnt_no_scan++ > 3) sleeptime_ms = 10*60*1000; //Sleep-Time nach  n Versuchen verlängern
			if (rtc_cnt_no_scan   > 6) sleeptime_ms = 60*60*1000;

			esp_sleep_enable_timer_wakeup(sleeptime_ms * 1000);
		    rtc_gpio_isolate(GPIO_NUM_15); //Ruhestrom bei externem Pulldown reduzieren
		    esp_deep_sleep_disable_rom_logging();
		    printf("Goto DeepSleep for %dms\n", sleeptime_ms);
			esp_deep_sleep_start();

		} else {
			ESP_ERROR_CHECK( esp_wifi_set_channel(rtc_wifi_channel, WIFI_SECOND_CHAN_NONE));
		}

	} else {
		rtc_wifi_channel = ch;
		ESP_ERROR_CHECK( esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE) );
	}
}



__attribute__((weak)) void init_rtc(void)  {
	//weak linkage
}

//Initialisierung als Sensor
bool wiog_sensor_init() {
	init_nvs();
	waked_up = false;

	esp_sleep_wakeup_cause_t rst_reason = esp_sleep_get_wakeup_cause();
#ifdef DEBUG_X
  printf("[%04d]RST-Cause: %d\n", now(), rst_reason);
#endif
    if ((rst_reason != ESP_SLEEP_WAKEUP_ULP)  &&
    	(rst_reason != ESP_SLEEP_WAKEUP_EXT1) &&
		(rst_reason != ESP_SLEEP_WAKEUP_TIMER)) {
    	//frisch initialisieren
    	rtc_wifi_channel = 0;
    	rtc_cycles = 0;
    	rtc_no_response_cnt = 0;
    	rtc_interval_ms = SENSOR_DEF_SLEEP_TIME_MS;
    	rtc_tx_pwr = MAX_TX_POWER;
    	//Systemvariablen aus NVS -> RTC-MEM
    	for (int ix=0; ix < MAX_SYSVAR; ix++) {
    		rtc_sysvar[ix] = nvs_get_sysvar(ix);
    	}
    	init_rtc();	//RTC-GPIO initialisieren
//    	init_ulp();	//ULP-Programm laden

    } else {
    	waked_up = true;
    }

    //bei wiederholt fehlendem Response -> ChannelScan erzwingen
    if (rtc_no_response_cnt > 2) rtc_wifi_channel = 0;

	//Rx-Queue -> Low_Prio Verarbeitung empfangener Daten
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
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
	ESP_ERROR_CHECK(esp_wifi_start());

	wifi_promiscuous_filter_t filter;
	filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
	ESP_ERROR_CHECK( esp_wifi_set_promiscuous_filter(&filter) );

	dev_uid = get_uid();	//Geräte-ID berechnen

	//Kanal setzen oder Channel-Scan
	wiog_set_channel(rtc_wifi_channel);
	esp_wifi_set_max_tx_power(rtc_tx_pwr);

	return waked_up;
}

// *********************************************************************************************

void app_main(void) {

	wiog_sensor_init();

	//Test-Frame senden ---------------------------------------------
	char txt[] = {"Hello World - How are you ? Dast ist ein Test"};

	uint8_t sz = strlen(txt) & 0xFF;
	uint8_t data[sz+1];				//Byte 0 => Längenbyte
	memcpy(&data[1], txt, sz);		//Byte 1 => Datenbereich
	data[0] = sz;

	/*bool b = */
	send_data_frame(data);

	//----------------------------------------------------------------

//	vTaskDelay (100*MS);
	rtc_onTime += esp_timer_get_time() / 1000;

	//Bereichsprüfung interval
	if (rtc_interval_ms == 0) rtc_interval_ms = SENSOR_DEF_SLEEP_TIME_MS;
	else if (rtc_interval_ms < SENSOR_MIN_SLEEP_TIME_MS) rtc_interval_ms = SENSOR_MIN_SLEEP_TIME_MS;
	else if (rtc_interval_ms > SENSOR_MAX_SLEEP_TIME_MS) rtc_interval_ms = SENSOR_MAX_SLEEP_TIME_MS;

	esp_sleep_enable_timer_wakeup(rtc_interval_ms * 1000);
    rtc_gpio_isolate(GPIO_NUM_15); //Ruhestrom bei externem Pulldown reduzieren
    esp_deep_sleep_disable_rom_logging();
    printf("Goto DeepSleep for %dms\n", rtc_interval_ms);
	esp_deep_sleep_start();

}

