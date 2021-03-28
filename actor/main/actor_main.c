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

#define VERSION  3
#define REVISION 0


#ifndef DEBUG_X
#define NIB_BC_INTERVAL_MS	60000	//NIB 1x je Minute an andere Nodes weiterleiten
#define SNR_INFO_INTERVAL_MS 5000
#else
#define	NIB_BC_INTERVAL_MS	10000
#define SNR_INFO_INTERVAL_MS 5000
#endif


payload_t tx_payload;
payload_t rx_payload;
//int ixtxpl = 0;



#undef  LOG_LOCAL_LEVEL	//Warnhinweis vermeiden
#define LOG_LOCAL_LEVEL ESP_LOG_NONE	//s. readme.txt
//zusätzlich Bootloader-msg  mit GPIO_15 -> low unterdrücken

// --------------------------------------------------------------------------------

uint32_t actual_frame_id; //ID (Random) des Tx-Paketes -> f. warten auf Antwort-Frame
uint32_t acked_frame_id;

species_t species = ACTOR;
uint8_t slot = MAX_SLOTS;

//SemaphoreHandle_t semph_wfa = NULL;	//Wait for ACK

//Daten
uint8_t   wifi_channel;
uint16_t  cnt_no_response;
uint32_t  cycle;
dev_uid_t my_uid; //Geräte-ID wird aus efuse_MAC berechnet

//Standard-Interval für Actoren
uint32_t interval_ms = 60*1000;

int64_t timer;	//TEST !!!!!!!

//WIoG - Wireless Internet of Garden
#define WIOG_RX_QUEUE_SIZE 12
static xQueueHandle wiog_rx_queue;

#define WIOG_TX_QUEUE_SIZE 6
static xQueueHandle wiog_tx_queue;

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

//Prototypen
//static void repeat_frame_to_gw_task (void *pvParameter);
//bool is_fid_handled(uint32_t fid);
//void add_fid(uint32_t fid);
void print_nib();


// ----------------------------------------------------------------------------------------------


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
	frame.timestamp = esp_timer_get_time();	//Empfangszeitpunkt (sinnhaft ??)

	if (xQueueSend(wiog_rx_queue, &frame, portMAX_DELAY) != pdTRUE) {
			ESP_LOGW("wiog_rx_cb: ", "receive queue fail");
			free(frame.data);
	}
}

//verzögertes Senden von Datenpaketen (Node-Slots)
void cb_tx_delay_slot(void* arg) {  //one-shot-timer
	wiog_event_txdata_t* ptx_frame = arg;
	//ACK wurde für diese FID bereits empfangen
	bool b1 = (ptx_frame->wiog_hdr.vtype == DATA_TO_GW) && (acked_fid == ptx_frame->wiog_hdr.frameid);

	if (!b1)
		if (xQueueSend(wiog_tx_queue, ptx_frame, portMAX_DELAY) != pdTRUE)
			ESP_LOGW("Tx-Queue: ", "Channelscan fail");

	free(ptx_frame);
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

		// Antwort auf eigenen Channel-Scan ---------------------------------
		// Auswertung der eigenen UID nicht erforderlich
		if (wifi_channel == 0) {
			if (pHdr->vtype == ACK_FOR_CHANNEL) {
				wifi_channel = pHdr->channel;	//Arbeitskanal wird in jedem WIOG-Header geliefert
			}
		}
		else

		//Empfang einer Kanalanfrage --------------------------------------------------------------------------------------
		//nur im Repeatermode antworten
		if ((pHdr->vtype == SCAN_FOR_CHANNEL) && (species == REPEATER))	{
			//Channel-Info an Device zurückliefern ---------------------------------
			//Antwort an Device im Repeaterslot n*5ms -> HiResTimer1
			//ohne Paket-Wiederholung, kein Ack von Device erwartet
			wiog_event_txdata_t* ptx_frame = malloc(sizeof(wiog_event_txdata_t)); //in cb freigeben !
			ptx_frame->crypt_data = false;
			ptx_frame->target_time = 0;
			ptx_frame->tx_max_repeat = 0;	//kein Response v. Device
			ptx_frame->data_len = 0;
			ptx_frame->data = NULL;
			ptx_frame->wiog_hdr = evt.wiog_hdr;
			ptx_frame->wiog_hdr.mac_from[5] = REPEATER;
			ptx_frame->wiog_hdr.mac_to[5] = evt.wiog_hdr.mac_from[5];
			ptx_frame->wiog_hdr.vtype = ACK_FOR_CHANNEL;
			ptx_frame->wiog_hdr.frameid = 0;	//keine Auswertung einer Frame-ID

			const esp_timer_create_args_t timer_args = {
  	  			  .callback = &cb_tx_delay_slot,
				  .arg = (void*) ptx_frame,  	//Tx-Frame über Timer-Callback in die Tx-Queue stellen
				  .name = "scan_ackn_slot"
			};

			esp_timer_handle_t h_timer1;
			ESP_ERROR_CHECK(esp_timer_create(&timer_args, &h_timer1));	//Create HiRes-Timer
			ESP_ERROR_CHECK(esp_timer_start_once(h_timer1, (slot+1) * SLOT_TIME_US)); 	// Start the timer, in Slot(0) antwortet der GW

			//SNR-Info an GW senden ---------------------------
			//Info-Pakt im Repeater-Slot
			//kein ACK erwartet
			wiog_event_txdata_t* ptx_frame2 = malloc(sizeof(wiog_event_txdata_t)); //in cb freigeben !
			ptx_frame2->crypt_data = false;
			ptx_frame2->target_time = 0,
			ptx_frame2->tx_max_repeat = 0;
			ptx_frame2->data_len = 0;
			ptx_frame2->data = NULL;
			ptx_frame2->wiog_hdr = evt.wiog_hdr;
			ptx_frame2->wiog_hdr.mac_from[5] = REPEATER;
			ptx_frame2->wiog_hdr.mac_to[5] = GATEWAY;
			ptx_frame2->wiog_hdr.uid = my_uid;				//eigene uid an GW
			//Nutzdaten - SNR direkt im Header senden
			ptx_frame2->wiog_hdr.vtype = SNR_INFO_TO_GW;
			ptx_frame2->wiog_hdr.tagA = evt.rx_ctrl.rssi - evt.rx_ctrl.noise_floor;	//Rx - SNR von device
			ptx_frame2->wiog_hdr.tagC = evt.wiog_hdr.uid;	//uid des devices - info


			const esp_timer_create_args_t timer_args2 = {
  	  			  .callback = &cb_tx_delay_slot,
				  .arg = (void*) ptx_frame2,  // argument will be passed to cb-function
				  .name = "scan_snr_info"
			};

			esp_timer_handle_t h_timer2;
			ESP_ERROR_CHECK(esp_timer_create(&timer_args2, &h_timer2));	//Create HiRes-Timer
			ESP_ERROR_CHECK(esp_timer_start_once(h_timer2, 5000 + slot * 5000));  //slot(0) => 5ms / slot(1) => 10ms ...
		}	// Ende Kanalanfrage ----------------------------------------------------------------

		else
		//Empfang eine NIB Broadcast von GW oder anderem Node
		if ((pHdr->vtype == BC_NIB) && (species == REPEATER)) {
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
				if (cbc_decrypt(evt.data, payload, blocksz, key, sizeof(key)) == 0) {
					//Management-Header des Gateway
					management_t* pGw_hdr = malloc(sizeof(management_t));
					memcpy(pGw_hdr, payload, sizeof(management_t));

					//Systemzugehörigkeit prüfen
					if (pGw_hdr->sid == SYSTEM_ID) {
						//Datenbereich im Anschluss an Management-Data
						node_info_block_t *pnib = (node_info_block_t*) &payload[sizeof(management_t)];

						if (pnib->ts > nib.ts){ //Aktualitätsprüfung
							bzero(&nib, sizeof(node_info_block_t));
							memcpy(&nib, pnib, evt.data_len);
							slot = get_node_slot(&nib, my_uid);
printf("UID: %d >> Slot: %d\n", evt.wiog_hdr.uid, slot);
print_nib();
						}
					}
				}
			}
		}	//if BC_NIB

		else
		//DataFrame Sensor/Actor/Node ==> GW
		//SNR zwischenspeichern (Rx-Quality am Node)
		//Prüfen, ob Node Prio zur Weiterleitung hat (NIB)
		//Node-Hopping verhindern
		if ((pHdr->vtype == DATA_TO_GW) && (species == REPEATER) && (pHdr->mac_from[5] != REPEATER)) {
			snr_buf[ix_snr_buf].dev_uid = evt.wiog_hdr.uid;
			snr_buf[ix_snr_buf].snr = evt.rx_ctrl.rssi - evt.rx_ctrl.noise_floor;
			ix_snr_buf++;
			if (ix_snr_buf == MAX_SNR_BUF_ENTRIES){
				ix_snr_buf = 0; 	//Überlauf verhindern -> überschreiben
				ov_snr_buf = true;	//Overflow-Flag setzen -> kompletten Puffer verarbeiten
			}
			//Daten wieder in die Queue stellen
			int ix = nib_get_priority(&nib, evt.wiog_hdr.uid, my_uid);
			wiog_event_txdata_t* ptx_frame = malloc(sizeof(wiog_event_txdata_t));
			uint8_t* data = malloc(evt.data_len);
			memcpy(data, evt.data, evt.data_len);
			ptx_frame->wiog_hdr = evt.wiog_hdr;
			ptx_frame->crypt_data = false;	//Verschlüsselung nicht ändern
			ptx_frame->target_time = 0;		//obsolete
			ptx_frame->tx_max_repeat = 0;		//keine Wiederholung repeateter Frames
			ptx_frame->wiog_hdr.tagC = my_uid;	//info zur freien Verwendung
			ptx_frame->wiog_hdr.tagB = evt.wiog_hdr.seq_ctrl; //info
			ptx_frame->data_len = evt.data_len;
			ptx_frame->data = data;
			ptx_frame->wiog_hdr.mac_from[5] = REPEATER;

			//über HiRes-Timer in die Queue stellen
			const esp_timer_create_args_t timer_args = {
	  	  		  .callback = &cb_tx_delay_slot,
				  .arg = (void*) ptx_frame,  // argument will be passed to cb-function
				  .name = "scan_snr_info"
			};
			esp_timer_handle_t h_timer;
			ESP_ERROR_CHECK(esp_timer_create(&timer_args, &h_timer));	//Create HiRes-Timer
			ESP_ERROR_CHECK(esp_timer_start_once(h_timer, 5000 + ix * 5000));
		}

		else

		//ACK des GW durch Nodes mit Prio 0+1 broadcasten
		if ((pHdr->vtype == ACK_FROM_GW) && (species == REPEATER) && (pHdr->uid != my_uid ) && (pHdr->mac_from[5] == GATEWAY)) {
			acked_fid = pHdr->frameid;
			//Daten wieder in die Queue stellen
			int ix = nib_get_priority(&nib, evt.wiog_hdr.uid, my_uid);
			if (( ix >= 0 ) && (ix < 2)) {
				wiog_event_txdata_t* ptx_frame = malloc(sizeof(wiog_event_txdata_t)); //in cb freigeben !
				ptx_frame->wiog_hdr = evt.wiog_hdr;
				ptx_frame->crypt_data = false;	//Verschlüsselung nicht ändern
				ptx_frame->target_time = 0;		//obsolete
				ptx_frame->tx_max_repeat = 0;		//keine Wiederholung repeateter Frames
				ptx_frame->wiog_hdr.tagC = my_uid;	//info zur freien Verwendung
				ptx_frame->data_len = 0;			//Ack-Frame hat keine Daten
				ptx_frame->data = NULL;
				ptx_frame->wiog_hdr.mac_from[5] = REPEATER;
				//In Slot senden
				const esp_timer_create_args_t timer_args = {
	  	  			  .callback = &cb_tx_delay_slot,
					  .arg = (void*) ptx_frame,  	//Tx-Frame über Timer-Callback in die Tx-Queue stellen
					  .name = "bc_act_from_gw"
				};

				esp_timer_handle_t h_timer3;
				ESP_ERROR_CHECK(esp_timer_create(&timer_args, &h_timer3));	//Create HiRes-Timer
				ESP_ERROR_CHECK(esp_timer_start_once(h_timer3, (slot+1) * SLOT_TIME_US)); 	// Start the timer
			}
		}

		else
		//Frame an eigene UID adressiert
		if (pHdr->uid == my_uid ) {

			// ACK des GW auf aktuelle Frame-ID, Tx-Widerholungen stoppen
			if ((pHdr->vtype == ACK_FROM_GW) && (pHdr->frameid == tx_fid)) {
				//Tx-Wiederholungen stoppen
				tx_fid = 0;
				xSemaphoreGive(ack_timeout_Semaphore);

				//Interval-Info auf Plausibilität prüfen
				if ((pHdr->interval_ms >= ACTOR_MIN_SLEEP_TIME_MS) && (pHdr->interval_ms <= ACTOR_MAX_SLEEP_TIME_MS))
					interval_ms = pHdr->interval_ms;
				//bei Kanal-Abweichung Channel-Scan veranlassen
				if (wifi_channel != pHdr->channel) wifi_channel = 0;
			}
		}


/*
		else
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
						if ((iv >= ACTOR_MIN_SLEEP_TIME_MS ) && (iv <= ACTOR_MAX_SLEEP_TIME_MS))
							interval_ms = iv;
						else
							interval_ms = ACTOR_DEF_SLEEP_TIME_MS;

						//falls unterschiedliche Kanäle in Wiog-hdr und GW-hdr -> Channel-Scan (in Main-Loop veranlassen)
						if (pHdr->channel != pGw_hdr->wifi_channel)	wifi_channel = 0;

						//weitere Datenauswertung in main
//!!!!!!!!!!!!!						if (main_rx_data_cb != NULL) {	main_rx_data_cb(payload); }

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
// !!!!!!				if (main_rx_data_cb != NULL) {	main_rx_data_cb(payload); }
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
*/
		free(evt.data);
	}	//while
}


//Zentraler Task zum Senden jedes Frame-Typs
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

		((wiog_header_t*) buf)->seq_ctrl = 0;
		//max Wiederholungen bis ACK von GW oder Node
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
	}
}

// Datenverarbeitung ----------------------------------------------------------------------------

//Datenframe managed an Gateway senden
void send_data_frame(payload_t* buf, uint16_t len, species_t spec) {

	uint8_t *data = (uint8_t*)buf;	//Datenbereich

	wiog_event_txdata_t tx_frame;
	tx_frame.wiog_hdr = wiog_get_dummy_header(GATEWAY, spec);
	tx_frame.wiog_hdr.uid = my_uid;
	tx_frame.wiog_hdr.species = spec;
	tx_frame.wiog_hdr.vtype = DATA_TO_GW;
	tx_frame.wiog_hdr.frameid = esp_random();
	tx_frame.tx_max_repeat = 5;					//max Wiederholungen, ACK erwartet

	tx_frame.data = malloc(len);
	memcpy(tx_frame.data, data, len);
	tx_frame.data_len = len;
	tx_frame.crypt_data = true;
	tx_frame.target_time = 0;

	if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE)
		ESP_LOGW("Tx-Queue: ", "Tx Data fail");

//	free(tx_frame.data);	//Feigabe hier korrekt ?
}


//Eintragen der Management-Daten in den Payload
//Aufrufer aktualisiert später die Anzahl der Datensätze
void set_management_data (management_t* pMan) {
	pMan->sid = SYSTEM_ID;
	pMan->uid = my_uid;
	pMan->wifi_channel = wifi_channel;
	pMan->version = VERSION;
	pMan->revision = REVISION;
	pMan->cycle = cycle;
	pMan->cnt_no_response = cnt_no_response;
	int8_t pwr;
	esp_wifi_get_max_tx_power(&pwr);
	pMan->cnt_entries = 0;
//	ixtxpl = 0;
}

//aktuellen NodeInfoBlock an andere Nodes im Netz verteilen, kein ACK erwartet
//incl. Managementdaten
void broadcast_nib() {
	//Länge der Nutzdatenblöcke
	int len_man = sizeof(management_t);
	//NIB nur aktive Device-Einträge
	int len_nib = 	sizeof(int64_t) + 					//ts
					sizeof(uint16_t) +					//dev_cnt
					sizeof(dev_uid_t) * MAX_SLOTS +		//Slot_Array
					(sizeof(dev_uid_t) + sizeof(node_info_t) * MAX_NODES ) * nib.dev_cnt;	//aktive Device-Zeilen

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
	tx_frame.data = malloc(len);
	//ManagementData voranstellen (f. Auswertung SID)
	management_t man;
	set_management_data(&man);
	memcpy(&tx_frame.data[0], &man, sizeof(management_t));
	memcpy(&tx_frame.data[len_man], &nib, len_nib);
	//Gesamt-Daten-Länge
	tx_frame.data_len = len;
	tx_frame.target_time = 0;

	//Datenpaket in Tx-Queue stellen, queued BY COPY !
	//data wird in tx_processing_task freigegeben
	if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE)
		ESP_LOGW("Tx-Queue: ", "Tx Data fail");

}

//in Node-Betrieb Zeitgesteurt eigenen NIB anderen Nodes anbieten
IRAM_ATTR void wiog_nib_spread_task(void *pvParameter) {
	while (true) {
		vTaskDelay(NIB_BC_INTERVAL_MS * MS);
		broadcast_nib();
	}
}

//Node sendet 1x je min Status-Frame mit Rx-Quality der empfangenen Devices
IRAM_ATTR void wiog_snr_info_task(void *pvParameter) {
	uint8_t loop = 0;

	while (true) {
		vTaskDelay(SNR_INFO_INTERVAL_MS * MS);

		//SNR-Info senden - zyklisch oder kurz vor Overflow oder Overflow bereits erfolgt
		if ((loop == 6) || (ix_snr_buf > MAX_SNR_BUF_ENTRIES - 3) || ov_snr_buf) {
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
			send_data_frame(&pl, pl.ix + sizeof(pl.man), REPEATER);
			//Buffer reset
			ix_snr_buf = 0;
			ov_snr_buf = false;
			loop = 0;

//hexdump((uint8_t*)&pl, pl.ix + sizeof(pl.man) );
		}
		loop++;
	}
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
			.target_time = 0
		};
		tx_frame.wiog_hdr = wiog_get_dummy_header(GATEWAY, ACTOR);
		tx_frame.wiog_hdr.vtype = SCAN_FOR_CHANNEL;
		tx_frame.wiog_hdr.uid = my_uid;
		tx_frame.wiog_hdr.species = ACTOR;	//Scan als Actor
		tx_frame.wiog_hdr.frameid = 0;
		ack_id = 1;	//Abbruch in Tx-Task verhindern

		for (ch = 1; ch <= wifi_country_de.nchan; ch++) {
			ESP_ERROR_CHECK( esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE));
			tx_frame.wiog_hdr.channel = ch;
			//Tx-Frame in Tx-Queue stellen
			if (xQueueSend(wiog_tx_queue, &tx_frame, portMAX_DELAY) != pdTRUE)
					ESP_LOGW("Tx-Queue: ", "Scan fail");
			vTaskDelay(50*MS);
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


void app_main(void) {

	//Initialisierung --------------------------------------------------------------------------------
	nvs_flash_init();

    cnt_no_response = 0;
    wifi_channel = 0;

    species = REPEATER;				//testweise als Repeater initialisieren !!!!!!!!!!!!!!!!!
    slot = 1;						//legt später der GW fest

    interval_ms = ACTOR_DEF_SLEEP_TIME_MS;

	//Rx-Queue -> Verarbeitung empfangener Daten
	wiog_rx_queue = xQueueCreate(WIOG_RX_QUEUE_SIZE, sizeof(wiog_event_rxdata_t));
	xTaskCreate(wiog_rx_processing_task, "wiog_rx_task", 4096, NULL, 12, NULL);

	//Tx-Queue - Unterprogramme stellen zu sendende Daten in die Queue
	wiog_tx_queue = xQueueCreate(WIOG_TX_QUEUE_SIZE, sizeof(wiog_event_txdata_t));
	xTaskCreate(wiog_tx_processing_task, "wiog_tx_task", 4096, NULL, 5, NULL);

	ack_timeout_Semaphore = xSemaphoreCreateBinary();
	if (species == REPEATER) {
		xTaskCreate(wiog_nib_spread_task, "wiog_nib_spread_task", 4096, NULL, 2, NULL);
		xTaskCreate(wiog_snr_info_task, "wiog_snr_info_task", 4096, NULL, 2, NULL);
	}
	//Liste zu Geräteverwaltung löschen
	nib_clear_all(&nib);

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

	my_uid = get_uid();	//Geräte-ID berechnen

	//Wifi-Kanal-Scan
	wiog_set_channel(wifi_channel);
	//actor sendet immer mit voller Leistung
	ESP_ERROR_CHECK( esp_wifi_set_max_tx_power(MAX_TX_POWER));

    // Ende Initialisierung ----------------------------------------------------------------------


//	wiog_rx_register_cb(rx_data_cb);	löschen
	printf("Actor-UID: %d\n", my_uid);

	while (true) {
		//ggf Channel-Scan vweranlassen
		if (wifi_channel == 0) wiog_set_channel(0);

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
		send_data_frame(&pl, pl.ix + sizeof(pl.man), ACTOR);

		//----------------------------------------------------------------


		vTaskDelay(interval_ms / portTICK_PERIOD_MS);

	}	//While
}

// -----------------------------------------------------------------------------------
/*
//Ringpuffer, mehrfache Bearbeitung von FID verhindern
#define FID_RBUF_SZ 8
uint8_t fid_rbuf_ix = 0;	//Index Schreibposition
uint32_t fid_rbuf[FID_RBUF_SZ];


//wurde die FID schonmal verarbeitet ?
//Ringpuffer abfragen
bool is_fid_handled(uint32_t fid){
	bool res = false;
	for (int i = 0; i < FID_RBUF_SZ; i++) {
		if (fid_rbuf[i] == fid) {
			res = true;
			break;
		}
	}
	return res;
}

//FID in Ringpuffer einfügen
void add_fid(uint32_t fid) {
	fid_rbuf[fid_rbuf_ix] = fid;
	fid_rbuf_ix++;
	fid_rbuf_ix &= FID_RBUF_SZ-1;
}
*/
//----------------------------------------------------------------------------------------

void print_nib() {
	printf("NIB TS: %lld DevCnt: %d\n", nib.ts, nib.dev_cnt);
	//hexdump((uint8_t*) nib.dev_info, evt.data_len);
	//hexdump((uint8_t*) nib.slot_info, sizeof(nib.slot_info));
	for (int i=0; i<MAX_SLOTS; i++) printf("%05d | ", nib.slot_info[i]);
	printf("\n");
	for (int i=0; i<nib.dev_cnt; i++){
		printf("%05d: ", nib.dev_info[i].dev_uid);
		for (int j=0; j<MAX_NODES; j++)
			printf(" -> %05d/%d", nib.dev_info[i].node_infos[j].node_uid, nib.dev_info[i].node_infos[j].snr);
		printf("\n");
	}
}
