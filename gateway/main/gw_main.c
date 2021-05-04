/**
	WIOG - Wireless Internet Of Garden
	Modul Gateway
	WIOG - Protokollabwicklung
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "string.h"
#include "mbedtls/aes.h"
#include "esp32/rom/crc.h"

#include "gw_main.h"
#include "rs232.h"



#define VERSION  3
#define REVISION 0
// --------------------------------------------------------------------------------



//uint16_t  cnt_no_response;

//uint32_t actual_frame_id;

//WIoG - Wireless Internet of Garden
#define WIOG_RX_QUEUE_SIZE 12
static xQueueHandle wiog_rx_queue;

#define WIOG_TX_QUEUE_SIZE 6
static xQueueHandle wiog_tx_queue;

//SemaphoreHandle_t return_timeout_Semaphore = NULL;
SemaphoreHandle_t ack_timeout_Semaphore = NULL;

dev_uid_t my_uid = 0;

uint32_t ack_id = 0;		//Vergleich mit FrameID
uint32_t tx_fid;			//aktuelle ID der letzten Sendung

int64_t ts_ack = 0;
uint8_t test = 0;

// Prototypen
// ------------
//Weiterleitung doppelte Frame-ID an RPi verhindern
void set_dbls_fid(uint32_t fid);
bool is_dbls_fid(uint32_t fid);

//Q-Frame via UART, Sofortmeldung SNR nach Device-Channelscan
void snr_info_to_uart(dev_uid_t nuid, dev_uid_t duid, uint8_t snr);
void bc_nib_immediately();
uint32_t get_def_sleep_time_ms(uint8_t species);	//SlepTimes aus DeviceInfoBlock ermitteln, ggf Default
uint32_t get_interval_ms(dev_uid_t uid, species_t spec);
//---------------------------------------------------------------------------------------------------------------

//Wifi-Rx-Callback im Sniffermode - Daten in die Rx-Queue stellen
IRAM_ATTR  void wiog_receive_packet_cb(void* buff, wifi_promiscuous_pkt_type_t type)
{
	const wifi_promiscuous_pkt_t   *ppkt = (wifi_promiscuous_pkt_t *)buff;
	const wiog_data_frame_t  *ipkt = (wiog_data_frame_t *)ppkt->payload;
	const wiog_header_t *header =  &ipkt->header;

	//nur fehlerfreie Pakete des eigenen Netzes bearbeiten
	if ((ppkt->rx_ctrl.rx_state != 0) || (memcmp(header->mac_net, &mac_net, sizeof(mac_addr_t)) !=0)) return;

	wiog_event_rxdata_t frame;
	memcpy(&frame.rx_ctrl, &ppkt->rx_ctrl, sizeof(wifi_pkt_rx_ctrl_t));
	memcpy(&frame.wiog_hdr, header, sizeof(wiog_header_t));
	frame.data_len = ppkt->rx_ctrl.sig_len - sizeof(wiog_header_t) - 4; //ohne FCS
	frame.data = malloc(frame.data_len);
	memcpy(frame.data, &ipkt->data, frame.data_len);
	frame.timestamp = esp_timer_get_time();	//Empfangszeitpunkt (sinnhaft ??)

	if (xQueueSend(wiog_rx_queue, &frame, portMAX_DELAY) != pdTRUE) {
			ESP_LOGW("GW", "receive queue fail");
			free(frame.data);
	}
}


//verzögertes Senden von Datenpaketen (Node-Slots)
void cb_tx_delay_slot(void* arg) {  //one-shot-timer
	wiog_event_txdata_t* ptx_frame = arg;
	if (xQueueSend(wiog_tx_queue, ptx_frame, portMAX_DELAY) != pdTRUE) {
			ESP_LOGW("Tx-Queue: ", "Channelscan fail");
	}
	free(ptx_frame);
}


//Verarbeitung eines empfangenen Datenpaketes
static void wiog_rx_processing_task(void *pvParameter) {
	wiog_event_rxdata_t evt;

	while ((xQueueReceive(wiog_rx_queue, &evt, portMAX_DELAY) == pdTRUE)) {

		wifi_pkt_rx_ctrl_t *pRx_ctrl = &evt.rx_ctrl;
		wiog_header_t *pHdr = &evt.wiog_hdr;
//		uint8_t data_len = pRx_ctrl->sig_len - sizeof(wiog_header_t) - 4;

		//GW_UID, Dev_UID, SNR
		snr_info_to_uart(my_uid, evt.wiog_hdr.uid, evt.rx_ctrl.rssi - evt.rx_ctrl.noise_floor);

		//Antwort auf ChannelScan eines Devices -------------------------------------------
		//Arbeitskanal wird in tx-processing in Header eingefügt
		//GW hat immer höchste Prio bei Kanalanfragen
		if (pHdr->vtype == SCAN_FOR_CHANNEL) {
			wiog_event_txdata_t tx_frame = {
				.crypt_data = false,
				.target_time = 0,
				.data_len = 0,
				.data = NULL,
			};
			tx_frame.wiog_hdr = evt.wiog_hdr;
			tx_frame.wiog_hdr.mac_from[5] = GATEWAY;
			tx_frame.wiog_hdr.mac_to[5] = evt.wiog_hdr.mac_from[5];
			tx_frame.wiog_hdr.vtype = ACK_FOR_CHANNEL;
			tx_frame.wiog_hdr.frameid = 0;
			tx_frame.tx_max_repeat = 0;
			ack_id = 1;
			if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE) {
				ESP_LOGW("Tx-Queue: ", "Channelscan fail");
			}
			bc_nib_immediately();	//I-Fame an UART
		} //scan_for_channel --------------------------------------------------------------

		else
		//Datenpaket von Device auswerten
		if  ((pHdr->vtype == DATA)) {
			//Ack an Device senden
			wiog_event_txdata_t* ptx_frame = malloc(sizeof(wiog_event_txdata_t));
			ptx_frame->crypt_data = false,
			ptx_frame->target_time = 0,
			ptx_frame->data_len = 0,
			ptx_frame->data = NULL,
			//Header modifiziert als ACK zurücksenden
			ptx_frame->wiog_hdr = evt.wiog_hdr;
			ptx_frame->wiog_hdr.mac_from[5] = GATEWAY;
			ptx_frame->wiog_hdr.mac_to[5] = evt.wiog_hdr.species;
			ptx_frame->wiog_hdr.vtype = ACK;
			ptx_frame->tx_max_repeat = 0;
			//Gerätespezifisches Interval zurückliefern
			ptx_frame->wiog_hdr.interval_ms = get_interval_ms(pHdr->uid, pHdr->species);

			//ACK 2ms verzögren
			const esp_timer_create_args_t timer_args = {
  	  			  .callback = &cb_tx_delay_slot,
				  .arg = (void*) ptx_frame,  	//Tx-Frame über Timer-Callback in die Tx-Queue stellen
				  .name = "bc_act_from_gw"
			};

			esp_timer_handle_t h_timer;
			ESP_ERROR_CHECK(esp_timer_create(&timer_args, &h_timer));	//Create HiRes-Timer
			ESP_ERROR_CHECK(esp_timer_start_once(h_timer, 2000)); 	// Start the timer

			//per UART an RPi-GW
			if (!is_dbls_fid(evt.wiog_hdr.frameid)) { //nur wenn Frame-ID noch nicht behandelt wurde
				set_dbls_fid(evt.wiog_hdr.frameid);	//FID in Liste eintragen
				//Daten via UART an RPi-GW senden	A-Frame
				//Datenblock entschlüsseln
				uint8_t buf[evt.data_len]; //decrypt Data nicht größer als encrypted Data
				bzero(buf, evt.data_len);
				if (wiog_decrypt_data(evt.data, buf, evt.data_len, evt.wiog_hdr.frameid) == 0)
					//kompletten Daten-Payload decrypted an RPi
					send_uart_frame(buf, evt.data_len, 'A');
				else logE("Dercrpt Error");
			}
		}	// Data To GW ------------------------------------------

		else
		//Sofortmeldung eines Node - SNR nach Channelscan	-> Q-Frame an RPi
		if (pHdr->vtype == SNR_INFO_TO_GW) {
			//node_UID, Dev_UID, SNR
			snr_info_to_uart(evt.wiog_hdr.uid, evt.wiog_hdr.tagC, evt.wiog_hdr.tagA);
			bc_nib_immediately();
		}// SNR Info To GW

		else
		//ACK eines Device empfangen
		if ((pHdr->vtype == ACK) && (pHdr->frameid == tx_fid)) {
			//Timestamp für Tx-Delay
			ts_ack = esp_timer_get_time();
			// ACK des GW auf aktuelle Frame-ID, Tx-Widerholungen stoppen
			//Tx-Wiederholungen stoppen
			tx_fid = 0;
			xSemaphoreGive(ack_timeout_Semaphore);
		}

		free(evt.data);
	}	//while


}

//Senden eines Datenpaketes
void wiog_tx_processing_task(void *pvParameter) {

	wiog_event_txdata_t evt;

	while ((xQueueReceive(wiog_tx_queue, &evt, portMAX_DELAY) == pdTRUE)) {

		uint16_t tx_len = get_blocksize(evt.data_len, AES_KEY_SZ) + sizeof(wiog_header_t);
		uint8_t buf[tx_len];
		bzero(buf, tx_len);

		//Datenpaket immer it aktuellem Wifi-Channel senden
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

		//Semaphore resetten
		xSemaphoreTake(ack_timeout_Semaphore, 0);
		((wiog_header_t*) buf)->seq_ctrl = 0;
		//max Wiederholungen bis ACK von Device oder Node
		tx_fid = evt.wiog_hdr.frameid;
		for (int i = 0; i <= evt.tx_max_repeat; i++) {
			//Frame senden
			esp_wifi_80211_tx(WIFI_IF_STA, &buf, tx_len, false);
			if (evt.tx_max_repeat == 0) break;	//1x Tx ohne ACK
			//warten auf Empfang eines ACK
			if (xSemaphoreTake(ack_timeout_Semaphore, 50*MS) == pdTRUE) {
				break; //ACK empfangen -> Wiederholung abbrechen
			}

			((wiog_header_t*) buf)->seq_ctrl++ ;	//Sequence++
		}

		free(evt.data);
	} //while Queue
}



//Datenframe managed an Species (Actor) senden
void send_data_frame(uint8_t* buf, uint16_t len, dev_uid_t uid) {
//	wiog_event_txdata_t tx_frame;
	device_info_t* di = get_device_info(uid);
	if (di == NULL) {
		logLV("Unbekanntes Gerät - UID: : ", uid);
		return;
	}
	wiog_event_txdata_t* ptx_frame = malloc(sizeof(wiog_event_txdata_t));
	//Datenframes gehen nur an ACTOR
	ptx_frame->wiog_hdr = wiog_get_dummy_header(ACTOR, GATEWAY);
	ptx_frame->wiog_hdr.uid = uid;
	ptx_frame->wiog_hdr.species = GATEWAY;
	ptx_frame->wiog_hdr.vtype = DATA;
	ptx_frame->wiog_hdr.frameid = esp_random();
	ptx_frame->tx_max_repeat = 5;					//max Wiederholungen, ACK erwartet
	ptx_frame->wiog_hdr.interval_ms = get_interval_ms(uid, di->species);
	ptx_frame->data = malloc(len);
	memcpy(ptx_frame->data, buf, len);
	ptx_frame->data_len = len;
	ptx_frame->crypt_data = true;
	ptx_frame->target_time = 0;

	uint32_t dts = esp_timer_get_time() - ts_ack;
	if (dts > 20000) {
		//Datenpaket sofort in die Queue stellen
		if (xQueueSend(wiog_tx_queue, ptx_frame, portMAX_DELAY) != pdTRUE)
			ESP_LOGW("Tx-Queue: ", "Tx Data fail");
		free(ptx_frame);
	} else {
		//Antwort auf P-Frame -> Datenpaket verzögren
		const esp_timer_create_args_t timer_args = {
	  			  .callback = &cb_tx_delay_slot,
			  .arg = (void*) ptx_frame,  	//Tx-Frame über Timer-Callback in die Tx-Queue stellen
			  .name = "data_from_gw"
		};
		esp_timer_handle_t h_timer;
		ESP_ERROR_CHECK(esp_timer_create(&timer_args, &h_timer));	//Create HiRes-Timer
		ESP_ERROR_CHECK(esp_timer_start_once(h_timer, dts)); 	// Start the timer
	}
}

//Eintragen der Management-Daten in den Payload
//Aufrufer aktualisiert später die Anzahl der Datensätze
void set_management_data (management_t* pMan) {
	pMan->sid = SYSTEM_ID;
	pMan->uid = my_uid;
	pMan->wifi_channel = wifi_channel;
	pMan->version = VERSION;
	pMan->revision = REVISION;
	pMan->cycle = 0;
	pMan->cnt_no_response = 0; //cnt_no_response;
	int8_t pwr;
	esp_wifi_get_max_tx_power(&pwr);
	pMan->cnt_entries = 0;
//	ixtxpl = 0;
}


//aktuellen NodeInfoBlock im Netz verteilen, kein ACK erwartet
//len_nib in Bytes - abhängig vob der Anzahl der registrierten Devices
//incl. Managementdaten
void broadcast_nib(node_info_block_t* pnib) {
	//Länge der Nutzdatenblöcke
	int len_man = sizeof(management_t);
	int len_nib = 	sizeof(int64_t) + 	//ts
					sizeof(uint16_t) +	//dev_cnt
					sizeof(dev_uid_t) * MAX_SLOTS +
					sizeof(dev_info_t) * pnib->dev_cnt;

	//wiog_header setzen
	wiog_event_txdata_t tx_frame;
	tx_frame.wiog_hdr = wiog_get_dummy_header(DUMMY, GATEWAY);
	tx_frame.wiog_hdr.uid = my_uid;
	tx_frame.wiog_hdr.species = GATEWAY;
	tx_frame.wiog_hdr.vtype = BC_NIB;
	tx_frame.wiog_hdr.frameid = esp_random();
	tx_frame.crypt_data = true;		//NIB verschlüsselt
	tx_frame.tx_max_repeat = 0;		//keine Wiederholung = kein ACK erwartet
	int len = len_nib + len_man;
	tx_frame.data = malloc(len);
	//ManagementData voranstellen (f. Auswertung SID)
	management_t man;
	set_management_data(&man);
	memcpy(&tx_frame.data[0], &man, sizeof(management_t));

	memcpy(&tx_frame.data[len_man], pnib, len_nib);
	//Gesamt-Daten-Länge
	tx_frame.data_len = len;
	tx_frame.target_time = 0;

	//Datenpaket in Tx-Queue stellen, queued BY COPY !
	//data wird in tx_processing_task freigegeben
	if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE)
		ESP_LOGW("Tx-Queue: ", "Tx Data fail");

}

void app_main(void) {
	nvs_flash_init();
	esp_netif_init();

	//Status-LED
    gpio_set_direction(LED_BL, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(LED_GN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(LED_RT, GPIO_MODE_INPUT_OUTPUT);

    LED_STATUS_ON;
    LED_GN_OFF;
    LED_RT_OFF;

	//Rx-Queue -> Verarbeitung empfangener Daten
	wiog_rx_queue = xQueueCreate(WIOG_RX_QUEUE_SIZE, sizeof(wiog_event_rxdata_t));
	xTaskCreate(wiog_rx_processing_task, "wiog_rx_task", 4096, NULL, 8, NULL);

	//Tx-Queue - Unterprogramme stellen zu sendende Daten in die Queue
	wiog_tx_queue = xQueueCreate(WIOG_TX_QUEUE_SIZE, sizeof(wiog_event_txdata_t));
	xTaskCreate(wiog_tx_processing_task, "wiog_tx_task", 4096, NULL, 5, NULL);

//	xTaskCreate(wiog_manage_slots_task, "wiog_slots_task", 2048, NULL, 0, NULL);

	ack_timeout_Semaphore = xSemaphoreCreateBinary();

	//Liste zu Geräteverwaltung löschen
	nib_clear_all(&nib);

    //UART initialisieren
	uart0_init();
    //Create a task to handler UART event from ISR
    xTaskCreate(rx_uart_event_task, "rx_uart_event_task", 8192, NULL, 3, NULL);




    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
   	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_country(&wifi_country_de) ); // set country for channel range [1 .. 13]
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR));
	esp_wifi_set_promiscuous(true);
	esp_wifi_set_promiscuous_rx_cb(&wiog_receive_packet_cb);

	ESP_ERROR_CHECK( esp_wifi_start() );

    wifi_channel = 0;
    bzero(&dib, sizeof(dib));
    send_uart_frame(NULL, 0, 'H');	//Send Hello-Frame an RPi

    //warten bis RPi mit WorkinChannel auf Hello-Frame geantwortet hat
    do { vTaskDelay(50*MS); } while (wifi_channel == 0);
    //wifi-channel mit c-Frame (UART) setzen
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

	wifi_promiscuous_filter_t filter;
	filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
	ESP_ERROR_CHECK( esp_wifi_set_promiscuous_filter(&filter) );

	//Gateway mit max tx power
	ESP_ERROR_CHECK( esp_wifi_set_max_tx_power(MAX_TX_POWER));

	int8_t maxpwr;
	esp_wifi_get_max_tx_power(&maxpwr);
	printf("Set Tx-Power: %.2f dBm\n", maxpwr *0.25);

	my_uid = GW_UID; 	//get_uid();	//Geräte-ID berechnen
    logLV("Gateway booted - UID: ", my_uid);

    while (true) {

		payload_t pl;
		bzero(&pl, sizeof(payload_t));
		pl.ix = 0;
		pl.man.sid = SYSTEM_ID;
		pl.man.wifi_channel = wifi_channel;
		pl.man.uid = 34640;						//!!!!!

		add_entry_I32(&pl, 1, 2, 3, 12345678 );
		add_entry_I64(&pl, 11, 22, 33,  987654321);

		char txt[] = "ABCDEF";
		add_entry_str(&pl, 55, 77, txt );

		add_entry_I32(&pl, 111, 222, 255, 888888 );

//		send_data_frame((uint8_t*) &pl, ACTOR);

		vTaskDelay(15000*MS);
	}


}

// ---------------------------------------------------------------------------------

//Liste der letzten empfangenen Frame-ID
//doppelte Weiterleitung an RPi verhindern
//Ringpuffer
#define DBLS_FID_SIZE 16	// 2^n !!
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

// -------------------------------------------------------------------------------------------
//Q-Frame via UART, Sofortmeldung SNR nach Device-Channelscan
void snr_info_to_uart(dev_uid_t nuid, dev_uid_t duid, uint8_t snr) {
	struct  {
		dev_uid_t node_uid;	//meldender Node / GW
		dev_uid_t dev_uid;	//uid des anfragenden Gerätes
		uint8_t snr;		//SNR des Device am Node
	}snr_info;

	snr_info.node_uid = nuid;
	snr_info.dev_uid = duid;
	snr_info.snr = snr;
	send_uart_frame(&snr_info, sizeof(snr_info), 'Q');
}

void bc_nib_immediately() {
	send_uart_frame(NULL, 0, 'I');
}

// -------------------------------------------------------------------------------------------
//Device - Info - Block

//Retrun Zeiger auf Device-Info oder NULL
device_info_t* get_device_info(dev_uid_t uid) {
	int n = sizeof(dib.device_info) / sizeof(device_info_t);
	for (int i=0; i<n; i++)
		if (dib.device_info[i].uid == uid) return &dib.device_info[i];
	return NULL;
}


uint32_t get_def_sleep_time_ms(uint8_t species){
	uint32_t res = 0;
	switch (species) {
		case SENSOR:   	if (dib.def_sensor_interval_ms > 0) return dib.def_sensor_interval_ms; break;
		case ACTOR:    	if (dib.def_actor_interval_ms  > 0) return dib.def_actor_interval_ms;  break;
		case REPEATER: 	if (dib.def_node_interval_ms   > 0) return dib.def_node_interval_ms;   break;
	}
	return res;
}

//Interval eines Device bestimmen, aus DIB
uint32_t get_interval_ms(dev_uid_t uid, species_t spec) {
	device_info_t* pdi = get_device_info(uid);
	if (pdi != NULL)
		return pdi->interval_ms;
		else return get_def_sleep_time_ms(spec);
}


//----------------------------------------------------------------------
