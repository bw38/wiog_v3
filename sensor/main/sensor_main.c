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

#define VERSION  3
#define REVISION 0


#undef  LOG_LOCAL_LEVEL	//Warnhinweis vermeiden
#define LOG_LOCAL_LEVEL ESP_LOG_NONE	//s. readme.txt
//zusätzlich Bootloader-msg  mit GPIO_15 -> low unterdrücken

//Daten, die einen Deep-Sleep überstehen müssen
RTC_DATA_ATTR static uint8_t  rtc_wifi_channel;
RTC_DATA_ATTR static uint32_t rtc_cycles;
RTC_DATA_ATTR static uint32_t rtc_interval_ms;
RTC_DATA_ATTR static int8_t   rtc_tx_pwr;		//Steuerung Sendeleistung
RTC_DATA_ATTR static uint32_t rtc_cnt_no_scan;  //Fehlversuche Channelscan
RTC_DATA_ATTR static uint32_t rtc_no_response;  //Fehlversuche Übertragung
RTC_DATA_ATTR static uint8_t  rtc_no_response_serie; //Fehlversuche aufeinanderfolgend

RTC_DATA_ATTR static uint32_t rtc_onTime;
// --------------------------------------------------------------------------------
uint16_t my_uid; //Geräte-IS wird aus efuse_MAC berechnet
uint32_t actual_frame_id; //ID (Random) des Tx-Paketes -> f. warten auf Antwort-Frame
uint32_t acked_frame_id;

species_t species = SENSOR;
bool waked_up;

SemaphoreHandle_t semph_wfa = NULL;	//Wait for ACK
uint32_t ack_id = 0;		//Vergleich mit FrameID
uint32_t tx_fid;


int64_t timer;	//TEST !!!!!!!

//SystemVariablen zur Steurung der Sensoren
#define MAX_SYSVAR  8 //Systemvariablen [0..7], Data-Entries im Quittungspaket
RTC_DATA_ATTR static int32_t   rtc_sysvar[MAX_SYSVAR] __attribute__((unused)); 	//0..7

//WIoG - Wireless Internet of Garden
#define WIOG_RX_QUEUE_SIZE 12
static xQueueHandle wiog_rx_queue;

#define WIOG_TX_QUEUE_SIZE 6
static xQueueHandle wiog_tx_queue;

SemaphoreHandle_t ack_timeout_Semaphore = NULL;
SemaphoreHandle_t goto_sleep_Semaphore = NULL;
uint32_t interval_ms;
int8_t tx_pwr_dB;

//Prototypen
void set_dbls_fid(uint32_t fid);
bool is_dbls_fid(uint32_t fid);

//---------------------------------------------------------------------------------------------------------------

__attribute__((weak)) void rx_data_handler(uint8_t* buf, uint16_t len)  {
	//weak linkage
	printf("Rx DataBytes: %d\n", len);
//	if (len>256) len = 256;
//	hexdump(buf, len);
}


__attribute__((weak)) void init_rtc(void)  {
	//weak linkage
}

// --------------------------------------------------------------------------------------------------------------

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
uint32_t fid2 = ptx_frame->wiog_hdr.frameid;
	free(ptx_frame);
printf("B%04x\n", fid2);
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
		// ACK des GW auf einen Datenframe, Tx-Widerholungen stoppen
		if ((pHdr->vtype == ACK_FROM_GW) && (tx_fid == pHdr->frameid)) {
			//Interval-Info - Plausibilitätsprüfung vor Deep_Sleep
			interval_ms = pHdr->interval_ms;
printf("Rx-Interval: %d\n", interval_ms);
			//bei Kanal-Abweichung Channel-Scan veranlassen
			if (rtc_wifi_channel != pHdr->channel) rtc_wifi_channel = 0;
			//empfohlene Tx-Pwr-Korrektur
			tx_pwr_dB = pHdr->txpwr;
printf("PWR-Korr: %d\n", tx_pwr_dB);

			//Wiederholung stoppen
//			tx_fid = 0;
			xSemaphoreGive(ack_timeout_Semaphore);
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

		uint16_t tx_len = get_blocksize(evt.data_len, AES_KEY_SZ) + sizeof(wiog_header_t);
		uint8_t buf[tx_len];
		bzero(buf, tx_len);

		//Kanal und Tx-Power-Index im Header übertragen
//		wifi_second_chan_t ch2;
//		esp_wifi_get_channel(&evt.wiog_hdr.channel, &ch2);
//		esp_wifi_get_max_tx_power(&evt.wiog_hdr.txpwr);

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

			cbc_encrypt(evt.pdata, &buf[sizeof(wiog_header_t)], evt.data_len, key, sizeof(key));

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
			if (i > 2) esp_wifi_set_max_tx_power(MAX_TX_POWER);
			//Frame senden
			esp_wifi_80211_tx(WIFI_IF_STA, &buf, tx_len, false);
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




// Öffentliche Funktionen ================================================================================

//Datenframe managed an Gateway senden
void send_data_frame(payload_t* buf, uint16_t len) {

	uint8_t *data = (uint8_t*)buf;	//Datenbereich

	wiog_event_txdata_t tx_frame;
	tx_frame.wiog_hdr = wiog_get_dummy_header(GATEWAY, species);
	tx_frame.wiog_hdr.uid = my_uid;
	tx_frame.wiog_hdr.species = species;
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

//Eintragen der Management-Daten in den Payload
//Aufrufer aktualisiert später die Anzahl der Datensätze
//Prefix des verschlüsselten Datenblocks an RPi
void set_management_data (management_t* pMan) {
	pMan->sid = SYSTEM_ID;
	pMan->uid = my_uid;
	pMan->wifi_channel = rtc_wifi_channel;
	pMan->species = species;
	pMan->version = VERSION;
	pMan->revision = REVISION;
	pMan->cycle = rtc_cycles++;
	pMan->cnt_no_response = rtc_no_response;
	ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&pMan->tx_pwr));
	pMan->cnt_entries = 0;
	//freier Heap (Info)
	pMan->sz_heap = xPortGetFreeHeapSize();
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
			.pdata = NULL,
			.target_time = 0	//sofort senden
		};
		tx_frame.wiog_hdr = wiog_get_dummy_header(GATEWAY, species);
		tx_frame.wiog_hdr.vtype = SCAN_FOR_CHANNEL;
		tx_frame.wiog_hdr.uid = my_uid;
		tx_frame.wiog_hdr.species = species;

		rtc_tx_pwr = MAX_TX_POWER;
		ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(rtc_tx_pwr));

		for (ch = 1; ch <= wifi_country_de.nchan; ch++) {
			ESP_ERROR_CHECK( esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE));
			//Tx-Frame in Tx-Queue stellen
			if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE)
					ESP_LOGW("Tx-Queue: ", "Scan fail");
printf("Scan Channel: %d\n", ch);

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
    	rtc_wifi_channel = 0;	//Channel-scan veranlassen
    	rtc_cycles = 0;
    	rtc_no_response = 0;
    	rtc_no_response_serie = 0;
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

	my_uid = get_uid();	//Geräte-ID berechnen

	//Kanal setzen oder Channel-Scan
	wiog_set_channel(rtc_wifi_channel);

	ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(rtc_tx_pwr));

	return waked_up;
}

// *********************************************************************************************

void app_main(void) {

	wiog_sensor_init();

	//Test-Frame senden ---------------------------------------------
	char txt[] = {"Hello World - How are you ? Das ist ein Test"};

	uint8_t sz = strlen(txt) & 0xFF;
	uint8_t data[sz+1];				//Byte 0 => Längenbyte
	memcpy(&data[1], txt, sz);		//Byte 1 => Datenbereich
	data[0] = sz;

	payload_t pl;
	pl.ix = 0;
	set_management_data(&pl.man);
	add_entry_str (&pl, dt_txt_info, 0x55, txt);

	//Data to GW
	send_data_frame(&pl, pl.ix + sizeof(pl.man));

	//----------------------------------------------------------------

	//Antwort des Gateway/Node abwarten
	if (xSemaphoreTake(goto_sleep_Semaphore, 500*MS) != pdTRUE) {
		rtc_no_response++;
		rtc_no_response_serie++;
	} else {
		rtc_no_response_serie = 0;
	}

printf("No response:       %d\n", rtc_no_response);
printf("No response serie: %d\n", rtc_no_response_serie);

	//Bereichsprüfung interval
	if (interval_ms < SENSOR_MIN_SLEEP_TIME_MS) rtc_interval_ms = SENSOR_MIN_SLEEP_TIME_MS;
	else if (interval_ms > SENSOR_MAX_SLEEP_TIME_MS) rtc_interval_ms = SENSOR_MAX_SLEEP_TIME_MS;
	else rtc_interval_ms = interval_ms;

	//Sendeleistung im nächsten Zyklus
	//Begrenzung aud am -3dB / +6dB je Zyklus
	if (tx_pwr_dB < -3) tx_pwr_dB = -3;
	if (tx_pwr_dB >  6) tx_pwr_dB =  6;

	rtc_tx_pwr += tx_pwr_dB * 4;
	if (rtc_tx_pwr > MAX_TX_POWER) rtc_tx_pwr = MAX_TX_POWER;
	if (rtc_tx_pwr < MIN_TX_POWER) rtc_tx_pwr = MIN_TX_POWER;

printf("Goto DeepSleep for %dms\n", rtc_interval_ms);

	rtc_onTime += esp_timer_get_time() / 1000;
	esp_sleep_enable_timer_wakeup(rtc_interval_ms * 1000);
    rtc_gpio_isolate(GPIO_NUM_15); //Ruhestrom bei externem Pulldown reduzieren
    esp_deep_sleep_disable_rom_logging();
	esp_deep_sleep_start();

}

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


