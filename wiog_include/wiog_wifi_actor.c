/*
 * wiog_wifi_actor.c
 *
 *  Created on: 06.06.2021
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
#include "wiog_wifi_actor.h"


#ifndef DEBUG_X
#define NIB_BC_INTERVAL_MS	 60000	//NIB 1x je Minute an andere Nodes weiterleiten
#define SNR_INFO_INTERVAL_MS 30000
#else
#define	NIB_BC_INTERVAL_MS	 20000
#define SNR_INFO_INTERVAL_MS 30000
#endif



#define WIOG_RX_QUEUE_SIZE 12
static xQueueHandle wiog_rx_queue;

#define WIOG_TX_QUEUE_SIZE 6
static xQueueHandle wiog_tx_queue;

species_t species;

//Daten
uint8_t   wifi_channel;
uint8_t	  chx;
uint32_t  cnt_no_response;
uint32_t  cnt_no_response_serie;
uint32_t  cnt_tx_repeat;
uint32_t  cycles;
dev_uid_t my_uid; //Geräte-ID wird aus efuse_MAC berechnet

uint32_t interval_ms = 60*1000;		//Standard-Interval für Actoren

//uint32_t actual_frame_id; //ID (Random) des Tx-Paketes -> f. warten auf Antwort-Frame
//uint32_t acked_frame_id;

//Prioritäten und Slots (synchron in allen Nodes (incl gw))
node_info_block_t nib;

//Node sammelt Device-SNR und schickt Daten Zyklisch an GW --------------
//SNR-Buffer
#define MAX_SNR_BUF_ENTRIES 16
struct snr_buf_entry {	//einzelner Eintrag
	dev_uid_t dev_uid;
	uint8_t snr;
};
struct snr_buf_entry snr_buf[MAX_SNR_BUF_ENTRIES];	//Statisches Array
uint8_t ix_snr_buf = 0;		//Index aktuelle Position
bool ov_snr_buf = false;	//Overflow-Flag
//------------------------------------------------------------------------

SemaphoreHandle_t ack_timeout_Semaphore = NULL;

uint32_t ack_id = 0;		//Vergleich mit FrameID	- Tx-Wiederholungen
uint32_t acked_fid;			//Node-Wiederholung
uint32_t tx_fid;			//aktuelle ID der letzten Sendung

int64_t ts_tx = 0;			//ESP-Time der letzen Aussendung
uint32_t nib_bc_interval_ms;//NIB-Interval - Weiterverteilung


//Prototypen
void set_dbls_fid(uint32_t fid);
bool is_dbls_fid(uint32_t fid);
void set_species(species_t sp);
void print_nib();

//----------------------------------------------------------------------------------------

//Wifi-Rx-Callback im Sniffermode - Daten in die Rx-Queue stellen
IRAM_ATTR  void wiog_receive_packet_cb(void* buff, wifi_promiscuous_pkt_type_t type)
{
	const wifi_promiscuous_pkt_t   *ppkt = (wifi_promiscuous_pkt_t *)buff;
	const wiog_data_frame_t  *ipkt = (wiog_data_frame_t *)ppkt->payload;
	const wiog_header_t *header =  &ipkt->header;

	//nur fehlerfreie Pakete des eigenen Netzes bearbeiten
	if ((ppkt->rx_ctrl.rx_state != 0) || (memcmp(header->mac_net, &mac_net, 6) !=0)) return;
	//Integrität des Header prüfen
	if (crc16((uint8_t*)header, sizeof(wiog_header_t)-2) != header->hdr_sign) return;

	wiog_event_rxdata_t frame;
	memcpy(&frame.rx_ctrl, &ppkt->rx_ctrl, sizeof(wifi_pkt_rx_ctrl_t));
	memcpy(&frame.wiog_hdr, header, sizeof(wiog_header_t));
	frame.data_len = ppkt->rx_ctrl.sig_len - sizeof(wiog_header_t) - 4; //Datenläne ohne WIOG-Header und  FCS
	frame.pdata = (uint8_t*)malloc(frame.data_len);
	memcpy(frame.pdata, &ipkt->pdata, frame.data_len);
	frame.timestamp = esp_timer_get_time();	//Empfangszeitpunkt (sinnhaft ??)

	if (xQueueSend(wiog_rx_queue, &frame, portMAX_DELAY) != pdTRUE) {
			ESP_LOGW("wiog_rx_cb: ", "receive queue fail");
			free(frame.pdata);
	}
}


//verzögertes Senden von Datenpaketen (Node-Slots)
void cb_tx_delay_slot(void* arg) {		 //one-shot-timer
	wiog_event_txdata_t* ptx_frame = arg;
	//ACK wurde für diese FID bereits empfangen -> Daten nicht wiederholen
	bool b1 = ((ptx_frame->wiog_hdr.vtype == DATA_TO_GW) || (ptx_frame->wiog_hdr.vtype == DATA_FROM_GW))
			&& ((acked_fid == ptx_frame->wiog_hdr.frameid) || (acked_fid == 0));

	if (!b1){
		if (xQueueSend(wiog_tx_queue, ptx_frame, portMAX_DELAY) != pdTRUE)
			ESP_LOGW("Tx-Queue: ", "Channelscan fail");
	} else {
		free(ptx_frame->pdata);
	}
	//Timer-Ressourcen freigeben
	ESP_ERROR_CHECK(esp_timer_delete(ptx_frame->h_timer));
	//tx-frame freigeben, wurde zuvor in Queue kopiert
	free(ptx_frame);
}


void start_tx_delay_timer(uint32_t delay_us, wiog_event_txdata_t* ptx) {
	const esp_timer_create_args_t timer_args = {
		.callback = &cb_tx_delay_slot,
		.arg = (void*) ptx,  // argument will be passed to cb-function
		.name = "tx delay"
	};
	ESP_ERROR_CHECK(esp_timer_create(&timer_args, &ptx->h_timer));	//Create HiRes-Timer
	ESP_ERROR_CHECK(esp_timer_start_once(ptx->h_timer, delay_us));
}


//Beliebiges Datenpaket weiterleiten
void repeat_frame(wiog_event_rxdata_t evt) {
	//nur Nodes mit Prio 1 & 2 des jeweiligen Device verwenden
	int slot = nib_get_priority(&nib, evt.wiog_hdr.uid, my_uid);
	if (slot > 2) return;
	//falls letzer Tx < min time -> Verschiebung in Slot 3 od. 4
	if (esp_timer_get_time() - ts_tx < 15000) slot += 2;

	//Daten wieder in die Queue stellen
	wiog_event_txdata_t* ptx_frame = malloc(sizeof(wiog_event_txdata_t));
	uint8_t* pdata = malloc(evt.data_len);
	memcpy(pdata, evt.pdata, evt.data_len);
	ptx_frame->wiog_hdr = evt.wiog_hdr;
	ptx_frame->crypt_data = false;	//Verschlüsselung nicht ändern
	ptx_frame->target_time = 0;		//obsolete
	ptx_frame->tx_max_repeat = 0;		//keine Wiederholung repeateter Frames
	ptx_frame->wiog_hdr.tagC = my_uid;	//info zur freien Verwendung
	ptx_frame->wiog_hdr.tagB = evt.wiog_hdr.seq_ctrl; //info
	ptx_frame->data_len = evt.data_len;
	ptx_frame->pdata = pdata;
	ptx_frame->wiog_hdr.mac_from[5] = REPEATER;
	ptx_frame->h_timer = NULL;
	ptx_frame->wiog_hdr.tagA = slot; //Testinfo f. Sniffer
	//über HiRes-Timer in die Tx-Queue stellen
	start_tx_delay_timer(slot * SLOT_TIME_US, ptx_frame);
}


//Verarbeitung eines empfangenen Datenpaketes
//kompletter Sniffer-Frame im Event-Parameter der Queue
//Payload muss in Queue-Loop freigegeben werden
IRAM_ATTR static void wiog_rx_processing_task(void *pvParameter)
{
	wiog_event_rxdata_t evt;

	while ((xQueueReceive(wiog_rx_queue, &evt, portMAX_DELAY) == pdTRUE)) {

		wiog_header_t *pHdr = &evt.wiog_hdr;

		//Empfang einer Kanalanfrage --------------------------------------------------------------------------------------
		//nur im Node-Mode antworten
		if ((pHdr->vtype == SCAN_FOR_CHANNEL) && (species == REPEATER))	{
			//Channel-Info an Device zurückliefern ---------------------------------
			//Antwort an Device im Repeaterslot n*5ms -> HiResTimer1
			//ohne Paket-Wiederholung, kein Ack von Device erwartet
			wiog_event_txdata_t* ptx_frame = malloc(sizeof(wiog_event_txdata_t)); //in cb freigeben !
			ptx_frame->crypt_data = false;
			ptx_frame->target_time = 0;
			ptx_frame->tx_max_repeat = 0;	//kein Response v. Device
			ptx_frame->data_len = 0;
			ptx_frame->pdata = NULL;
			ptx_frame->wiog_hdr = evt.wiog_hdr;
			ptx_frame->wiog_hdr.mac_from[5] = REPEATER;
			ptx_frame->wiog_hdr.mac_to[5] = evt.wiog_hdr.mac_from[5];
			ptx_frame->wiog_hdr.vtype = ACK_FOR_CHANNEL;
			ptx_frame->wiog_hdr.frameid = 0;	//keine Auswertung einer Frame-ID
			//SNR-Info für GW
			ptx_frame->wiog_hdr.tagA = evt.rx_ctrl.rssi - evt.rx_ctrl.noise_floor;	//Rx - SNR von device
			ptx_frame->wiog_hdr.tagC = my_uid; //node-UID
			ptx_frame->h_timer = NULL;

			int slot = nib_get_node_slot(&nib, my_uid);
			start_tx_delay_timer(slot * SLOT_TIME_US, ptx_frame);

		}	// Ende Kanalanfrage ----------------------------------------------------------------

		else
		//Empfang eines NIB Broadcast von GW oder anderem Node
		if ((pHdr->vtype == BC_NIB) && (species == REPEATER)) {
			int blocksz = evt.data_len;
			if (blocksz > 0) {
				//Datenblock entschlüsseln
				uint8_t buf[evt.data_len]; //decrypt Data nicht größer als encrypted Data
				payload_t* ppl = (payload_t*)buf;
				if ((wiog_decrypt_data(evt.pdata, buf, evt.data_len, evt.wiog_hdr.frameid) == 0) &&
					//integrität des Datenblocks prüfen
					(ppl->man.len <= MAX_DATA_LEN) &&
					(ppl->man.crc16 == crc16((uint8_t*)&ppl->man.len, ppl->man.len-2))) {
						node_info_block_t *pnib = (node_info_block_t*) &buf[sizeof(management_t)];
						if (pnib->ts > nib.ts){ //Aktualitätsprüfung
							nib_clear_all(&nib);
							if (evt.data_len > sizeof(node_info_block_t)) evt.data_len = sizeof(node_info_block_t);
							memcpy(&nib, pnib, evt.data_len);
							nib_bc_interval_ms = nib.bc_interval_sek * 1000;

							#ifdef DEBUG_X
								print_nib();
							#endif
						}
				} else {
					printf("NIB-Data Error\n");
				}
			}
		}	//if BC_NIB

		else

		//Repeater - Daten - Funktionen ---------------------------------------
		//Repeater - Hopping unterbinden
		//keine Pakete an eigene UID repeaten
		if	((species == REPEATER) && (pHdr->mac_from[5] != REPEATER) && (pHdr->uid != my_uid )) {
				//DataFrame Sensor/Actor/Node ==> GW
				//SNR zwischenspeichern (Rx-Quality am Node)
				if (pHdr->vtype == DATA_TO_GW) {
					if (pHdr->mac_from[5] != GATEWAY) { //SNR v.Actor/Sensor
						snr_buf[ix_snr_buf].dev_uid = evt.wiog_hdr.uid;
						snr_buf[ix_snr_buf].snr = evt.rx_ctrl.rssi - evt.rx_ctrl.noise_floor;
						ix_snr_buf++;
						if (ix_snr_buf == MAX_SNR_BUF_ENTRIES){
							ix_snr_buf = 0; 	//Überlauf verhindern -> überschreiben
							ov_snr_buf = true;	//Overflow-Flag setzen -> kompletten Puffer verarbeiten
						}
					}
					repeat_frame(evt);
				}
				else
				if (pHdr->vtype == DATA_FROM_GW) {
					repeat_frame(evt);
				}

				else
				//ACK broadcasten
				if ((pHdr->vtype == ACK_TO_GW) || (pHdr->vtype == ACK_FROM_GW))  {
					acked_fid = pHdr->frameid;
					repeat_frame(evt);
				}
		}	//Ende Repeaterfunktionen -----------------------------------


		else
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

				if (pHdr->species == REPEATER) set_species(REPEATER);
				else set_species(ACTOR);

				(*cb_rx_ack_handler)(pHdr);	//Callback actor_main
			}

			else
			//Reset
			if (pHdr->vtype == RESET_DEV) {
				printf("Restart now\n");
				esp_restart();
			}

			else
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
				ptx_frame->h_timer = NULL;
				//eigene Sendeleistungs-Ix an GW liefern
				esp_wifi_get_max_tx_power(&ptx_frame->wiog_hdr.txpwr);

				//ACK 2ms verzögren
				start_tx_delay_timer(2000, ptx_frame);

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
			wiog_encrypt_data(evt.pdata, &buf[sizeof(wiog_header_t)], evt.data_len, evt.wiog_hdr.frameid);
		} else {
			memcpy(&buf[sizeof(wiog_header_t)], evt.pdata, evt.data_len);
		}


		((wiog_header_t*) buf)->seq_ctrl = 0;
		//max Wiederholungen bis ACK von GW oder Node
		tx_fid = evt.wiog_hdr.frameid;
		for (int i = 0; i <= evt.tx_max_repeat; i++) {
			//Header-Signature
			((wiog_header_t*)buf)->hdr_sign = crc16(buf, sizeof(wiog_header_t)-2);
			//Frame senden
			esp_wifi_80211_tx(WIFI_IF_STA, buf, tx_len, false);
			ts_tx = esp_timer_get_time();
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
			//bei fehlendem ACK
			((wiog_header_t*) buf)->seq_ctrl++ ;	//Sequence++
			if (i == evt.tx_max_repeat) {
				cnt_no_response++;
				cnt_no_response_serie++;
			}
			set_species(ACTOR);	//wird in späterem ACK-Frame ggf wieder auf Repeater gesetzt
		}
		cnt_tx_repeat += ((wiog_header_t*) buf)->seq_ctrl;
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
	pMan->species = species;
	pMan->cycles = ++cycles;
	pMan->ulp_cycles = 0;
	pMan->onTime = esp_timer_get_time() / 1000000;	//sek
	pMan->cnt_no_response = cnt_no_response;
	pMan->cnt_tx_repeat = cnt_tx_repeat;
	pMan->cnt_no_response = cnt_no_response;
	ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&pMan->tx_pwr));
	pMan->sz_heap = 0;	//mqtt-Ausgabe unterdrücken
	#ifdef DEBUG_X
	//freier Heap (Test Mem-Leaks)
	pMan->sz_heap = xPortGetFreeHeapSize();
	#endif
	pMan->cnt_entries = 0;
}

//Datenframe managed an Gateway senden
void send_data_frame(payload_t* buf, uint16_t len, species_t spec) {

	//unverschlüsselten Payload absichern
	buf->man.len = len;
	buf->man.crc16 = crc16((uint8_t*)&buf->man.len, len-2);

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
	tx_frame.h_timer = NULL;

	if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE)
		ESP_LOGW("Tx-Queue: ", "Tx Data fail");
}

// Node - Information - Block ----------------------------------------------------------------


//aktuellen NodeInfoBlock an andere Nodes im Netz verteilen, kein ACK erwartet
//incl. Managementdaten
void broadcast_nib() {
	if (nib.ts == 0) return; //nur weiter, wenn gültiger NIB empfangen wurde
	//Länge der Nutzdatenblöcke
	int len_man = sizeof(management_t);
	//NIB nur aktive Device-Einträge
	int len_nib = nib_get_size(&nib);
	//wiog_header setzen
	wiog_event_txdata_t tx_frame;
	tx_frame.wiog_hdr = wiog_get_dummy_header(DUMMY, REPEATER);
	tx_frame.wiog_hdr.uid = my_uid;
	tx_frame.wiog_hdr.species = REPEATER;
	tx_frame.wiog_hdr.vtype = BC_NIB;
	tx_frame.wiog_hdr.frameid = esp_random();
	tx_frame.crypt_data = true;		//NIB verschlüsselt
	tx_frame.tx_max_repeat = 0;		//keine Wiederholung = kein ACK erwartet
	int len = len_nib + len_man;
	tx_frame.pdata = malloc(len);
	//ManagementData voranstellen (f. Auswertung SID)
	management_t man;
	set_management_data(&man);
	memcpy(&tx_frame.pdata[0], &man, sizeof(management_t));
	memcpy(&tx_frame.pdata[len_man], &nib, len_nib);
	//Gesamt-Daten-Länge
	tx_frame.data_len = len;
	tx_frame.target_time = 0;
	tx_frame.h_timer = NULL;

	//unverschlüsselten Payload absichern
	management_t* pman = (management_t*)tx_frame.pdata;
	pman->len = len;
	pman->crc16 = crc16((uint8_t*)&pman->len, len-2);

	//Datenpaket in Tx-Queue stellen, queued BY COPY !
	//data wird in tx_processing_task freigegeben
	if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE)
		ESP_LOGW("Tx-Queue: ", "Tx Data fail");

}

//in Node-Betrieb Zeitgesteurt eigenen NIB anderen Nodes anbieten
IRAM_ATTR void wiog_nib_spread_task(void *pvParameter) {
	while (true) {
		vTaskDelay(nib_bc_interval_ms * MS);
		if (species == REPEATER) broadcast_nib();
	}
}

//Node sendet Status-Frame mit Rx-Quality der empfangenen Devices
IRAM_ATTR void wiog_snr_info_task(void *pvParameter) {
	#define LOOP_DIV 6
	uint8_t loop = 0;
	while (true) {
		vTaskDelay(SNR_INFO_INTERVAL_MS / LOOP_DIV * MS);
		//SNR-Info senden - zyklisch oder kurz vor Overflow oder Overflow bereits erfolgt
		if ((loop == LOOP_DIV) || (ix_snr_buf > MAX_SNR_BUF_ENTRIES - 3) || ov_snr_buf) {
			//falls Overflow registriert wurde -> gesamten Puffer verwenden
			if (ov_snr_buf) ix_snr_buf = MAX_SNR_BUF_ENTRIES -1;
			//Datensätze als Payload
			payload_t pl;
			set_management_data(&pl.man);
			pl.ix = 0;		//playload immer erst initialisieren
			//SNR-Info als I32-Data je Eintrag
			for (int i = 0; i < ix_snr_buf; i++)
				add_entry_I32(&pl, dt_snr_info, 0, snr_buf[i].snr, snr_buf[i].dev_uid);

			//Data to GW
			if ( species == REPEATER )
				//absenden als Actor, um Dataframe von anderen Nodes wiederholen lassen zu können
				send_data_frame(&pl, pl.ix + sizeof(pl.man), ACTOR);

			//Buffer reset
			ix_snr_buf = 0;
			ov_snr_buf = false;
			loop = 0;
		}
		loop++;
	}
}

// Channel - Scan -----------------------------------------------------------------------------
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
			.target_time = 0
		};
		tx_frame.wiog_hdr = wiog_get_dummy_header(GATEWAY, ACTOR);
		tx_frame.wiog_hdr.vtype = SCAN_FOR_CHANNEL;
		tx_frame.wiog_hdr.uid = my_uid;
		tx_frame.wiog_hdr.species = ACTOR;	//Scan als Actor
		tx_frame.wiog_hdr.frameid = 0;
		tx_frame.h_timer = NULL;
		ack_id = 1;	//Abbruch in Tx-Task verhindern

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
    cnt_tx_repeat = 0;
    //Channel-Scan veranlassen
    wifi_channel = 0;

    //Liste zu Node-Verwaltung löschen
    nib_clear_all(&nib);

    interval_ms = ACTOR_DEF_SLEEP_TIME_MS;	//wird in ACK-Frame zugewiesen
    nib_bc_interval_ms = NIB_BC_INTERVAL_MS;


	//Rx-Queue -> Verarbeitung empfangener Daten
	wiog_rx_queue = xQueueCreate(WIOG_RX_QUEUE_SIZE, sizeof(wiog_event_rxdata_t));
	xTaskCreate(wiog_rx_processing_task, "wiog_rx_task", 4096, NULL, 8, NULL);

	//Tx-Queue - Unterprogramme stellen zu sendende Daten in die Queue
	wiog_tx_queue = xQueueCreate(WIOG_TX_QUEUE_SIZE, sizeof(wiog_event_txdata_t));
	xTaskCreate(wiog_tx_processing_task, "wiog_tx_task", 4096, NULL, 9, NULL);

	ack_timeout_Semaphore = xSemaphoreCreateBinary();

	//Tasks für Repeater-Mode
	xTaskCreate(wiog_nib_spread_task, "wiog_nib_spread_task", 4096, NULL, 2, NULL);
	xTaskCreate(wiog_snr_info_task, "wiog_snr_info_task", 4096, NULL, 2, NULL);

	wifi_country_t country = wifi_country_de;
    esp_netif_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_country(&country) );
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


//zentrale Funktion zum Setzen der Species
void set_species(species_t sp) {
	#ifdef DEBUG_X
		if (sp != species) printf("Set Species: %d\n", sp);
	#endif
	species = sp;
}


void print_nib() {
	printf("NIB TS: %lld iv: %d DevCnt: %d\n", nib.ts, nib.bc_interval_sek, nib.dev_cnt);
	//hexdump((uint8_t*) nib.dev_info, evt.data_len);
	//hexdump((uint8_t*) nib.slot_info, sizeof(nib.slot_info));
	for (int i=0; i<MAX_SLOTS; i++) printf("%05d | ", nib.slot_info[i]);
	printf("\n");
	for (int i=0; i<nib.dev_cnt; i++){
		printf("%05d:%02ddB ", nib.dev_info[i].dev_uid, nib.dev_info[i].best_snr);
		for (int j=0; j<2; j++)
			printf(" -> %05d", nib.dev_info[i].node_uid[j]);
		printf("\n");
	}
}

