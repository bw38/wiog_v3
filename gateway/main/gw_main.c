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


uint8_t wifi_channel = WORKING_CHANNEL;
uint16_t  cnt_no_response;

//uint32_t actual_frame_id;

//WIoG - Wireless Internet of Garden
#define WIOG_RX_QUEUE_SIZE 12
static xQueueHandle wiog_rx_queue;

#define WIOG_TX_QUEUE_SIZE 6
static xQueueHandle wiog_tx_queue;

SemaphoreHandle_t return_timeout_Semaphore = NULL;
dev_uid_t my_uid = 0;
uint32_t ack_id = 0;		//Vergleich mit FrameID


// Prototypen
// ------------
//Weiterleitung doppelte Frame-ID an RPi verhindern
void set_dbls_fid(uint32_t fid);
bool is_dbls_fid(uint32_t fid);

//Q-Frame via UART, Sofortmeldung SNR nach Device-Channelscan
void snr_info_to_uart(dev_uid_t nuid, dev_uid_t duid, uint8_t snr);


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


//Verarbeitung eines empfangenen Datenpaketes
static void wiog_rx_processing_task(void *pvParameter) {
	wiog_event_rxdata_t evt;

	while ((xQueueReceive(wiog_rx_queue, &evt, portMAX_DELAY) == pdTRUE)) {

		wifi_pkt_rx_ctrl_t *pRx_ctrl = &evt.rx_ctrl;
		wiog_header_t *pHdr = &evt.wiog_hdr;
//		uint8_t data_len = pRx_ctrl->sig_len - sizeof(wiog_header_t) - 4;

		//Antwort auf ChannelScan eines Devices -------------------------------------------
		//Arbeitskanal wird in tx-processing in Header eingefügt
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
			//GW_UID, Dev_UID, SNR
			snr_info_to_uart(my_uid, evt.wiog_hdr.uid, evt.rx_ctrl.rssi - evt.rx_ctrl.noise_floor);

		} //scan_for_channel --------------------------------------------------------------
		else

		//Datenpaket von Device auswerten
		if (pHdr->vtype == DATA_TO_GW) {
			//Ack an Device senden
			wiog_event_txdata_t tx_frame = {
				.crypt_data = false,
				.target_time = 0,
				.data_len = 0,
				.data = NULL,
			};
			//Header modifiziert als ACK zurücksenden
			tx_frame.wiog_hdr = evt.wiog_hdr;
			tx_frame.wiog_hdr.mac_from[5] = GATEWAY;
			tx_frame.wiog_hdr.mac_to[5] = evt.wiog_hdr.mac_from[5];
			tx_frame.wiog_hdr.vtype = ACK_FROM_GW;
			tx_frame.tx_max_repeat = 0;
tx_frame.wiog_hdr.interval_ms = 15000;	//interval ais DeviceInfo ermitteln
//0 setzen wenn Sensor Standby
			//ACK sofort an Device senden
			if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE) {
				ESP_LOGW("Tx-Queue: ", "Tx ACK fail");
			}

			//Daten via UART an RPi-GW senden	A-Frame
			if (!is_dbls_fid(evt.wiog_hdr.frameid)) { //nur wenn Frame-ID noch nicht behandelt wurde
				set_dbls_fid(evt.wiog_hdr.frameid);	//FID in Liste eintragen

				//Datenblock entschlüsseln
				uint8_t buf[evt.data_len]; //decrypt Data nicht größer als encrypted Data
				bzero(buf, evt.data_len);
				if (wiog_decrypt_data(evt.data, buf, evt.data_len, evt.wiog_hdr.frameid) == 0)
					//kompletten Daten-Payload decrypted an RPi
					send_uart_frame(buf, evt.data_len, 'A');
				else logE("Dercrpt Error");
			}


		}

		//Sofortmeldung eines Node - SNR nach Channelscan
		//Q-Frame an RPi
		if (pHdr->vtype == SNR_INFO_TO_GW) {
			//node_UID, Dev_UID, SNR
			snr_info_to_uart(evt.wiog_hdr.uid, evt.wiog_hdr.tagC, evt.wiog_hdr.tagA);
		}

/*
		//Empfang eines Datenpaketes von Sensor/Actor
		if (pHdr->vtype == DATA_TO_GW) {
			//prüfen, ob Frame-ID bereits bekannt ist
			int dix = get_dbls(pHdr->frameid);
			bool new_id = false;
			if (dix < 0) {	//Frame zum ersten Mal empfangen
				//neuen Eintrag in Ringpuffer anlagen
				//Anzahl der unbearbeiteten Pakete darf nicht größer als der Ringpuffer werden
				bzero(&dbls[dbl_ix], sizeof(double_entry_t));
				dbls[dbl_ix].frameid = pHdr->frameid;	//Kennung des Datenpaketes
				dbls[dbl_ix].species = pHdr->species;	//Ursprungsspezies
				dbls[dbl_ix].uid = pHdr->uid;			//Ursprungs-UID
				dbls[dbl_ix].txpwr = pHdr->txpwr;			//rel Tx-Power des Ursprungsdevices

				dix = dbl_ix;							//f. weitere Einträge
				dbl_ix++;								//Index f. nächstes Pakes
				dbl_ix &= DBLS_SZ - 1;					//Index Ringpuffer
				new_id = true;
			}

			//max SNR in Double-List eintragen
			if (pHdr->mac_from[5] >= REPEATER){	//wiederholtes Paket?, SNR am 1. Repeater
				if(dbls[dix].max_snr < pHdr->tagA) {
					dbls[dix].max_snr = pHdr->tagA;	//werden im 1. Repeater der Kette gesetzt
					dbls[dix].mac = pHdr->tagC;
				}
			} else { //direkter Empfang
				int8_t snr = pRx_ctrl->rssi - pRx_ctrl->noise_floor;
				if(dbls[dix].max_snr < snr) {
					dbls[dix].max_snr = snr;
					dbls[dix].mac = GATEWAY;
				}
			}

			if (new_id) {	//Frame verarbeiten

				//Datenblock entschlüsseln
				uint8_t buf[evt.data_len];
				bzero(buf, evt.data_len);

				uint8_t key[] = {AES_KEY};	//CBC-AES-Key
				uint32_t u32 = evt.wiog_hdr.frameid;	// Frame-ID => 4 letzten Byres im Key
				key[28] = (uint8_t)u32;
				key[29] = (uint8_t)(u32>>=8);
				key[30] = (uint8_t)(u32>>=8);
				key[31] = (uint8_t)(u32>>=8);

				cbc_decrypt(evt.data, buf, evt.data_len, key, sizeof(key));
printf("%s\n", buf);
				uint32_t *pid = malloc(sizeof(uint32_t));
				*pid = pHdr->frameid;

				//RxQuittung senden & Verarbeitung der Rx-Daten
				//Test - Speicherplatz am Task-Ende wieder freigeben !!!
				xTaskCreate(rx_data_processing_task, "wiog_tx_data_task", 2048, pid, 2, NULL);

			}
		} //data_to_gw
*/
/*		//Empfangsbestätigung vom Actor
		if ((pHdr->vtype == RETURN_FROM_ACTOR)&&(actual_frame_id == pHdr->frameid)) {
			actual_frame_id++;	//verfälschen
			//Entschlüsselung nicht erforderlich ???

			//Empfang wurde bestätigt -> Tx- Widerholung abbrechen
			xSemaphoreGive(return_timeout_Semaphore);
		}
*/


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

		evt.wiog_hdr.seq_ctrl = 0;
		//max Wiederholungen bis ACK von GW oder Node
		for (int i = 0; i <= evt.tx_max_repeat; i++) {
			//Abbruch ab 2.Durchlauf falls ID bestätigt wurde
			if ((i > 0) && (ack_id == evt.wiog_hdr.frameid)) break;
			//Frame senden
			ESP_ERROR_CHECK(esp_wifi_80211_tx(WIFI_IF_STA, &buf, tx_len, false)); //tx_len 24 .. 1500
			evt.wiog_hdr.seq_ctrl++;
			//min. Ruhezeit zw. zwei Sendungen
			vTaskDelay(50*MS);
		}

		free(evt.data);
	} //while Queue
}



//Datenframe managed an Species (Sensor od. Actor) senden
void send_data_frame(uint8_t* buf, species_t species) {

	uint8_t len = buf[0];		//Längenbyte
	uint8_t *data = &buf[1];	//Datenbereich

	wiog_event_txdata_t tx_frame;
	tx_frame.wiog_hdr = wiog_get_dummy_header(species, GATEWAY);
	tx_frame.wiog_hdr.uid = my_uid;
	tx_frame.wiog_hdr.species = GATEWAY;
	tx_frame.wiog_hdr.vtype = DATA_TO_DEVICE;
	tx_frame.wiog_hdr.frameid = esp_random();
	tx_frame.tx_max_repeat = 5;					//max Wiederholungen, ACK erwartet

	tx_frame.data = malloc(len);
	memcpy(tx_frame.data, data, len);
	tx_frame.data_len = len;
	tx_frame.crypt_data = true;
	tx_frame.target_time = 0;

	if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE)
		ESP_LOGW("Tx-Queue: ", "Tx Data fail");

	free(tx_frame.data);	//Feigabe hier korrekt ?
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
	pMan->cnt_no_response = cnt_no_response;
	int8_t pwr;
	esp_wifi_get_max_tx_power(&pwr);
	pMan->cnt_entries = 0;
//	ixtxpl = 0;
}


//aktuellen NodeInfoBlock im Netz verteilen, kein ACK erwartet
//incl. Managementdaten
void broadcast_nib() {
	//Länge der Nutzdatenblöcke
	int len_man = sizeof(management_t);
	int len_nib = sizeof(node_info_block_t);

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
nib.ts = esp_timer_get_time();
nib.dev_info[0].dev_uid=0x1234;
nib.dev_info[0].node_uids[1]=0x5678;
nib.slot_info[1] = my_uid;
	memcpy(&tx_frame.data[len_man], &nib, len_nib);
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

	return_timeout_Semaphore = xSemaphoreCreateBinary();

	//Liste zu Geräteverwaltung löschen
	nib_clear_all(&nib);

    //UART initialisieren
	uart0_init();
    //Create a task to handler UART event from ISR
    xTaskCreate(rx_uart_event_task, "rx_uart_event_task", 4096, NULL, 12, NULL);


//	bzero(dbls, sizeof(dbls));

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
   	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );tstop(0);
	ESP_ERROR_CHECK( esp_wifi_set_country(&wifi_country_de) ); // set country for channel range [1 .. 13]
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR));
	esp_wifi_set_promiscuous(true);
	esp_wifi_set_promiscuous_rx_cb(&wiog_receive_packet_cb);

	ESP_ERROR_CHECK( esp_wifi_start() );


	esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE);
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

	wifi_promiscuous_filter_t filter;
	filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
	ESP_ERROR_CHECK( esp_wifi_set_promiscuous_filter(&filter) );

	//Gateway mit max tx power
	ESP_ERROR_CHECK( esp_wifi_set_max_tx_power(MAX_TX_POWER));
	int8_t maxpwr;
	esp_wifi_get_max_tx_power(&maxpwr);
	printf("Set Tx-Power: %.2f dBm\n", maxpwr *0.25);

	my_uid = get_uid();	//Geräte-ID berechnen
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

		broadcast_nib();

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
// *******************************************************************************************************
//Obsolete Funktionen / Variablen /etc

/*
// Double-List ----------------------------------------
//nur das erste empfangene Paket  einer ID wird verarbeitet, Wiederholungen werden verworfen
//bester Empfangspegel an Repeater / Gateway - zu Regulierung Tx-Power von Sensoren
typedef struct {
	uint32_t frameid;	//Frame - ID
	uint16_t uid;
	species_t species;
	int8_t txpwr;
	uint8_t max_snr; 	//bestes SNR GW oder Repeater
	uint8_t mac;	 	//MAC5 des GW/Rep mit höchstem SNR
} double_entry_t;

#define DBLS_SZ 16
double_entry_t dbls[DBLS_SZ];
uint8_t dbl_ix = 0;
//-----------------------------------------------------

uint8_t slot = 0;

//static void rx_data_processing_task(void *pvParameter);
int get_dbls(uint32_t fid);

*/

/*
//Double-Liste durchsuchen, ob Eintrag bereits vorhanden ist
//Return: ix des Eintrages, -1 wenn nicht vorhanden
int get_dbls(uint32_t fid) {
	int res = -1;
	for (int dix = 0; dix < DBLS_SZ; dix++ ){
		if (dbls[dix].frameid == fid) {
			res = dix;
			break;
		}
	}
	return res;
}
*/


//---------------------------------------------------------------------------------------------------
/*
//Rx-Quittung senden und Weiterleitung der Daten an PiGateway
static void rx_data_processing_task(void *pvParameter){
	uint32_t *id = (uint32_t*)pvParameter;

//	vTaskDelay(40*MS);
	int ix = get_dbls(*id);
	if (ix >=0) {
		wiog_event_txdata_t tx_frame;
		tx_frame.wiog_hdr = wiog_get_dummy_header(dbls[ix].species, GATEWAY);
		tx_frame.wiog_hdr.frameid = *id;
		tx_frame.wiog_hdr.uid = dbls[ix].uid;
		tx_frame.wiog_hdr.species = dbls[ix].species;
		tx_frame.wiog_hdr.vtype = RETURN_FROM_GW;
		tx_frame.wiog_hdr.tagB = dbls[ix].species;
		tx_frame.wiog_hdr.txpwr = IDEAL_SNR - dbls[ix].max_snr; //Einpegeln d. Tx-Power der Sensoren

printf("FID:%08x | UID:%05d | SP:%02d | SNR:%02d | MAC:%02x | PWR:%02d\n",
	dbls[ix].frameid, dbls[ix].uid, dbls[ix].species, dbls[ix].max_snr, dbls[ix].mac, dbls[ix].txpwr);

		dbls[ix].frameid++;	//ID verfälschen, um Wiederholung zu vermeiden

		tx_frame.target_time = 0;
		tx_frame.crypt_data = true;

		management_t man;
		man.sid = SYSTEM_ID;
		man.uid = dbls[ix].uid;
		man.wifi_channel = 3;
		man.cnt_entries = 0;
		man.interval = 6000;

		tx_frame.data_len = sizeof(management_t);
		tx_frame.data = malloc(tx_frame.data_len);

		memcpy(tx_frame.data, &man, tx_frame.data_len);

		if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE) {
			free(tx_frame.data);
			ESP_LOGW("Tx-Queue: ", "Return Data fail");
		}
	}
	free(id);
	vTaskDelete(NULL);
}
*/





// Slot - Management ---------------------------------------------------------------

/*
// Slot List ------------------------------------------
//Verwaltung der Repeater-Slots
typedef struct __attribute__((packed)){
	uint16_t uid;		//UID des epeaters
	int8_t   tout_cdwn;
} slot_entry_t;

#define SET_TOUT 10; //Startwert für CountDown-Timer

#define SLOTS_SZ 8	//max Anzhal von Repeatern im Netz
slot_entry_t slots[SLOTS_SZ];

//Mutex for Critical Sections
portMUX_TYPE slot_mutex = portMUX_INITIALIZER_UNLOCKED;

void wiog_slots_init() {
	portENTER_CRITICAL(&slot_mutex);
	for (int i=0; i<SLOTS_SZ; i++) {
		slots[i].uid = 0;
		slots[i].tout_cdwn = -1;
	}
	portEXIT_CRITICAL(&slot_mutex);
}

//Slot eines Repeaters (uid) ermitteln
//wenn nicht vorhanden -> neu anlegen
//wenn Liste schon voll ist -> return -1
int8_t wiog_get_slot(uint16_t uid) {
	int res = -1;
	int i;
	for (i=0; i< SLOTS_SZ; i++) {
		if (slots[i].uid == uid)
			break;
	}
	if (i==SLOTS_SZ) { //UID in Liste nicht vorhanden
		for (i=0; i<SLOTS_SZ; i++)
			if (slots[i].tout_cdwn == -1) { //freier Platz ??
				slots[i].uid = uid;
				break;
			}
	}
	if (i != SLOTS_SZ) {
		slots[i].tout_cdwn = SET_TOUT;
		res = i;
	}
	return res;
}

//Slots-Array zyklisch bereinigen
//läuft mit Prio 0
void wiog_manage_slots_task(void *pvParameter) {
	wiog_slots_init();

	while(true) {
		slot_entry_t slh[SLOTS_SZ]; 	//Hilfsliste f Thread-Sicehrheit
		portENTER_CRITICAL(&slot_mutex);
		memcpy(slh, slots, sizeof(slots));
		portEXIT_CRITICAL(&slot_mutex);

		for (int i = SLOTS_SZ-1; i>=0; i-- ) {
			if (slh[i].tout_cdwn >= 0) slh[i].tout_cdwn--;
			if (slh[i].tout_cdwn == 0) {
				slh[i].uid = 0;
				slh[i].uid = 0;
				if (i < SLOTS_SZ -1) 	//auf letzten Platz nicht schieben
					//dahinter liegende Plätze nach vorn schieben
					memcpy(&slh[i], &slh[i+1], (SLOTS_SZ-i-1) * sizeof(slot_entry_t));
				slh[SLOTS_SZ-1].uid = 0;
				slh[SLOTS_SZ-1].tout_cdwn = -1;
			}
		}

		//zurück kopieren
		portENTER_CRITICAL(&slot_mutex);
		memcpy(slots, slh, sizeof(slots));
		portEXIT_CRITICAL(&slot_mutex);
		vTaskDelay(5*1000*MS);
	} //while
}
*/
//----------------------------------------------------------------------
