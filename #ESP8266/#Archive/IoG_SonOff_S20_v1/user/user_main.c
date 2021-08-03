/*********************************************
 *
Pumpen - Ventil - Steueung, ESp8266
Datenübertragung ESP32-Gateway
*
 *********************************************/

/*
 Wichtig :im Makefile des Projects
 LINKFLAGS_eagle.app.v6 =
 ...
 -lespnow
 hinzufügen
 */


#include <limits.h>
#include "user_interface.h"
#include <osapi.h>
#include "espnow.h"
#include "gpio.h"
#include "../../esp32/IoG_Include/iog_sensors.h"



#define DEBUG_X 		//Debugmodus
#define DEBUG_XX		//selektives debuggen

#define IROM ICACHE_FLASH_ATTR	//for lazy writer

// *************************************************

uint16_t flags_sensors;

#define FW_VERSION				02
#define FW_REVISION				01
#define MIN_INTERVAL_MS			1*SEK
#define MAX_INTERVAL_MS			1*MIN
#define DEFAULT_INTERVAL_MS 	30*SEK
#define MY_SPECIES				SPECIES_ACTOR

#define S20_RELAIS		GPIO_ID_PIN(12)
#define LED_BL			GPIO_ID_PIN(13)  //LED bl fest mit Relaisfunktion verbunden
#define S20_LED_GN_ON	GPIO_OUTPUT_SET(LED_BL, 0)
#define S20_LED_GN_OFF	GPIO_OUTPUT_SET(LED_BL, 1)
#define S20_BUTTON		GPIO_ID_PIN(0)

os_timer_t scan_timer; 	//Timeout Channel-Scan
os_timer_t led_gn_delay_timer;
os_timer_t bt_debounce_timer;

//up / down aus Sicht des Actors
union payload_t  payload_tx;		//Daten zur Übertragung an Gateway

//Sensor-Identification
uint16_t myUID;	//Chip-default_mac -> crc16

uint32_t rst_reason;		//Reset-Grund (0-6 s. API Manual)

uint8_t  wifi_channel;
uint16_t cnt_no_response = 0;
int16_t  cntScan;
uint8_t  cnt_entries = 0; //Anzahl der individuellen Werte
uint32_t cycles = 0 ;
uint32_t cycle_gw = 0;
uint32_t interval_ms = DEFAULT_INTERVAL_MS ;
uint8_t  no_tx_f = 0; 			//nicht-senden-flag
uint8_t scan_channel;

uint32_t s20_bt_changed = 0;	//20 -> Taste gedrückt, 0-> kein Tastendruck

//Senden von Statusmeldungen (nach Schaltbefehl oder Interval)
#define		statusTx_TaskPrio       USER_TASK_PRIO_0
#define		statusTx_TaskQueueLen   5
os_event_t	statusTx_TaskQueue[statusTx_TaskQueueLen];
os_timer_t	statusTx_interval;
#define		SIG_TX_NO	0
#define 	SIG_TX_YES	1


void IROM led_gn_off(void);
int16_t IROM crc16(char *data_p, uint16_t length);
int now();
void ptime(char *text);

//==========================================================================

// ESPNOW --------------------------------------------------------------------------

//Paket vom Gateway empfangen
void espnow_recv_cb(u8 *macaddr, u8 *data, u8 len)
{
	union payload_t pl_rx;
	if (len > sizeof(pl_rx.data)) return; //Datenfehler
	os_bzero(pl_rx.data, sizeof(pl_rx.data));
	os_memcpy(pl_rx.data, data, len);

#ifdef DEBUG_X
	os_printf("[%04d] recv_cb: %d Bytes from %02X\n", now(), len, macaddr[5]);
#endif

	if (pl_rx.devUID == myUID)	//richtiger Adressat
	{
		cnt_no_response = 0;	//Fehlerzähler zurücksetzen
		if (wifi_channel != pl_rx.wifi_channel)
		{
			wifi_channel = pl_rx.wifi_channel;
			wifi_set_channel(wifi_channel);
		}
		os_timer_disarm(&scan_timer); // stop channel-scan;

		//Interval übernehmen (min/max-Prüfung)
		if (pl_rx.interval_ms != interval_ms)
		{
			if ((pl_rx.interval_ms > MIN_INTERVAL_MS -1) &&
				(pl_rx.interval_ms < MAX_INTERVAL_MS + 1))
				interval_ms = pl_rx.interval_ms;
			else
				interval_ms = DEFAULT_INTERVAL_MS;
			system_os_post(statusTx_TaskPrio, SIG_TX_NO, 0); //Timer ändern
		}

		//Steuerbefehl vom Gateway
		if (len > HEADER_SZ)	//Steuerbefehl vorhanden
		{
			if (cycle_gw != pl_rx.cycle)
			{
				cycle_gw = pl_rx.cycle;
				int ix;
				for (ix = 0; ix < MAX_ENTRIES; ix++)
				{
					if ((pl_rx.entries[ix].type == SW_MASK) && (pl_rx.entries[ix].status == 0))
					{
						uint32_t x = pl_rx.entries[ix].value;
						if (x == 1)
							GPIO_OUTPUT_SET(S20_RELAIS, 1); //Ausgang high-Aktiv
						else
							GPIO_OUTPUT_SET(S20_RELAIS, 0);
					}
				}
				system_os_post(statusTx_TaskPrio, SIG_TX_YES, 0); //Sende Status sofort
			}
		}
		else if (pl_rx.species == SPECIES_DUMMY)
			system_os_post(statusTx_TaskPrio, SIG_TX_YES, 0); //Sende Status sofort nach erstem Konzakt

		led_gn_off();	//verzögert ausschalten
	}
}


//Packet has sent
void espnow_send_cb(u8 *mac_addr, u8 status) {
	if (status == 0) {
#ifdef DEBUG_X
		os_printf("[%04d] send_cb from MAC: %d\n", now(), mac_addr[5]);
#endif
	}
}

// --------------------------------------------------------------------------------------

//*********************************************************************
//Steuerung beenden, Grundstellung herstellen
/*reason:
	0  - normales Ende
	-1 - Kanalscan abgebrochen
*/
void IROM StopIt(uint16_t reason)
{
	os_timer_disarm(&scan_timer);

	//Kanäle in Grundstellung
	GPIO_OUTPUT_SET(S20_RELAIS, 1);

	os_printf("Seuerung beendet" , reason);
}

//Grüne LED nachleuchten lassen
void IROM led_gn_off_timer_cb(void *arg)
{
	S20_LED_GN_OFF;
}

void IROM led_gn_off(void)
{
	os_timer_disarm(&led_gn_delay_timer);
	os_timer_setfn(&led_gn_delay_timer, (os_timer_func_t *) led_gn_off_timer_cb, (void *) 0);
	os_timer_arm(&led_gn_delay_timer, 200, false);
}

//*********************************************************************

// Device-Daten bereitstellen -------------------------------------------------------
//Zentrale Funktion zum hinzufügen von Datensätzen
static void Add_Entry(union data_entry_t dat)
{
	memcpy(&payload_tx.data[HEADER_SZ + cnt_entries * DATA_ENTRY_SZ], &dat, sizeof(dat));
	cnt_entries++;
}

//Verwaltungsdaten des Devices
//Struct wurde aus der Sensor-Kommunikation übernommen, Daten teilw. überflüssig
void IROM set_Management_Data()
{
	bzero(payload_tx.data, sizeof(payload_tx.data));
	cnt_entries = 0;
	payload_tx.devUID = myUID;
	payload_tx.version = FW_VERSION;
	payload_tx.revision = FW_REVISION;
	payload_tx.species = MY_SPECIES;
	payload_tx.wifi_channel = wifi_get_channel();
	payload_tx.cycle = cycles;
	payload_tx.interval_ms = interval_ms;
}


void IROM set_Device_Data()
{
	uint32_t x = GPIO_INPUT_GET(S20_RELAIS);

	union data_entry_t dex = {
		.type = SW_MASK,
		.status = 0,
		.value = x & 0x1
	};
	Add_Entry(dex);

	x = GPIO_INPUT_GET(S20_BUTTON);
	union data_entry_t dey = {
		.type = BUTTON_ONOFF,
		.status = 0,
		.value = (x^1) | s20_bt_changed	//Bit0-> aktueller Zustand; Bit1 durch Wechsel ausgelöst
	};
	Add_Entry(dey);
	s20_bt_changed = 0;	//BT-Flag rücksetzen

}

// Task - Status an Gateway senden
void IROM send_actor_status_task(os_event_t *e)
{
	if (wifi_channel == 0) return;
	S20_LED_GN_ON;
	os_timer_disarm(&statusTx_interval);
	if (cnt_no_response > 5 ){	//Timeout
		StopIt(-1);
		system_restart();
	}

	if (e->sig == SIG_TX_YES) { // == NO -> nur Timer neu setzen
		cycles++;
		set_Management_Data();
		set_Device_Data();

		uint8_t mac_gw[6];
		os_memcpy(mac_gw, mac_gateway, 6);
		uint8_t mac_re[6];
		os_memcpy(mac_re, mac_repeater, 6);
		esp_now_send(mac_gw, payload_tx.data, HEADER_SZ + cnt_entries* DATA_ENTRY_SZ);
		esp_now_send(mac_re, payload_tx.data, HEADER_SZ + cnt_entries* DATA_ENTRY_SZ);
		cnt_no_response++;
		ptime("Send Status-Frame\n");
	}
	//Timerinterval (neu) setzen bzw retriggern
	os_timer_arm(&statusTx_interval, interval_ms, TRUE);
}

//Interval-Timer, zyklische Statusmeldungen
void IROM statusTx_interval_cb(void *arg)
{
	system_os_post(statusTx_TaskPrio, SIG_TX_YES, 0);
}

void IROM scan_timer_cb (void *arg)
{
	os_timer_disarm(&scan_timer);
	if (wifi_channel == 0)
	{
		set_Management_Data();
		payload_tx.species = SPECIES_DUMMY; //ESP-Datapoints antworten di
		wifi_set_channel(scan_channel);
		//Header an Gateway und Repeater senden
		uint8_t mac_gw[6];
		os_memcpy(mac_gw, mac_gateway, 6);
		uint8_t mac_re[6];
		os_memcpy(mac_re, mac_repeater, 6);


		esp_now_send(mac_gw, payload_tx.data, HEADER_SZ);
		esp_now_send(mac_re, payload_tx.data, HEADER_SZ);

#ifdef DEBUG_X
		os_printf("[%04d] Send Dummy on CH: %d\n", now(), scan_channel);
#endif
		if (scan_channel++ > IOG_MAX_CHANNEL-1)
		{
			scan_channel = 1; //beginne von vorn
			if (++cntScan > 2) { // nach x Durchläufen
				cntScan = 0;
				//warte nach erfolglosem Scan
				os_timer_arm(&scan_timer, 10000, true);
				return;
			}
		}
		os_timer_arm(&scan_timer, 150, true);// retriggern 100ms je Kanal
	}
	else led_gn_off();
}

// ****************************************************************************************
//S20-Button
uint16_t cnt = 0;
void bt_debounce_timer_cb()
{
	//nächster Interrupt mit konträrem Schaltzustand
	if (GPIO_INPUT_GET(S20_BUTTON) == 1)
		gpio_pin_intr_state_set(S20_BUTTON, GPIO_PIN_INTR_LOLEVEL);
	else
		gpio_pin_intr_state_set(S20_BUTTON, GPIO_PIN_INTR_HILEVEL);
	s20_bt_changed = 0x02;	//Bit 1 - Flag f. Tastenbetätigung setzen
	system_os_post(statusTx_TaskPrio, SIG_TX_YES, 0);	//Status senden
}

void  bt_int_handler()
{
	gpio_pin_intr_state_set(S20_BUTTON, GPIO_PIN_INTR_DISABLE);
//	ETS_GPIO_INTR_DISABLE(); //nach Entprellzeit wieder freigeben
	uint32 gpio_status;
	gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
	//clear interrupt status
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);
	//debounce-timer
	os_timer_disarm(&bt_debounce_timer);
	os_timer_setfn(&bt_debounce_timer, (os_timer_func_t *) bt_debounce_timer_cb, (void *) 0);
	os_timer_arm(&bt_debounce_timer, 050, false);
}

/****************************************************************************************/

void IROM user_init(void) {

	wifi_station_set_auto_connect(0);	//kein Auto-Connect im Stationmode!!!

	ETS_GPIO_INTR_DISABLE(); // Disable gpio interrupts

	//BuiltIn LED f. Testzwecke - Output GPIO 2
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);	//Tx-Kontrolle, Buildin LED
	S20_LED_GN_ON;

	//Station-Mode
	wifi_set_opmode(STATION_MODE);

	//ESP now initialisieren
	BOOL bOk = false;
	if (esp_now_init() == 0) {
		bOk = true;

		esp_now_register_recv_cb(espnow_recv_cb);
		esp_now_register_send_cb(espnow_send_cb);

		//Sensor => Station-Mode => Controller-Mode
		esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);

		//Gegenstation(Ziel) registrieren => Gateway => Station-Mode = Controller-Mode
		uint8_t mac_gw[6];
		os_memcpy(mac_gw, mac_gateway, 6);
		uint8_t mac_re[6];
		os_memcpy(mac_re, mac_repeater, 6);

#ifdef ENCRYPTION_YES
//		esp_now_set_peer_key(mac_gw, pmk_now_key, 16);
		esp_now_add_peer(mac_gw, ESP_NOW_ROLE_CONTROLLER, 0, lmk_now_key, 16);
		esp_now_add_peer(mac_re, ESP_NOW_ROLE_CONTROLLER, 0, lmk_now_key, 16);
#else
		esp_now_add_peer(mac_gw, ESP_NOW_ROLE_CONTROLLER, 0, NULL, 0);
		esp_now_add_peer(mac_re, ESP_NOW_ROLE_CONTROLLER, 0, NULL, 0);
#endif
		os_printf("ESPnow Ok\n");
	} else
		os_printf("Fehler ESPnow\n");

	struct rst_info* rst = system_get_rst_info(); //rst->reason == 5 -> nach Wakeup
	rst_reason = rst->reason;

	//UID Device aus MAC-Addr des chips -> crc16 -> nicht wirklich unicate !!!
	uint8_t umac[6];
	wifi_get_macaddr(STATION_IF, umac);
	myUID = (uint16_t) crc16(umac, 6);
	os_printf("Sensor-UID: %05d\n" , myUID);

	//eigene mac setzen
	uint8_t mac[6];
	os_memcpy(mac, mac_actor, 6);
	wifi_set_macaddr(STATION_IF, mac);

	//S20-Relaisinitialisieren
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
	GPIO_OUTPUT_SET(S20_RELAIS, 0);	//Relais ausschalten

	//S20-Button initialisieren
    GPIO_DIS_OUTPUT(S20_BUTTON);                                 //Configure it in input mode.
    //lt Schaltung externer PullUp
    ETS_GPIO_INTR_DISABLE();                                     //Close the GPIO interrupt
    ETS_GPIO_INTR_ATTACH(bt_int_handler, NULL);                  //Register the interrupt function
    gpio_pin_intr_state_set(S20_BUTTON, GPIO_PIN_INTR_LOLEVEL);	//warten auf ersten Tastendruck
    ETS_GPIO_INTR_ENABLE() ;


	cntScan = 0; 		//f. max Scan-Versuche
	wifi_channel = 0;	//wifi-scan
	set_Management_Data();
	payload_tx.species = SPECIES_DUMMY; //ESP-Datapoints antworten direkt

	//Starte Scan auf Kanal 1
	uint8_t ch = 1;
	uint8_t loop_cnt = 0;

	//Kanal-Scan initialisieren
	scan_channel = 1;
	os_timer_disarm(&scan_timer);
	os_timer_setfn(&scan_timer, (os_timer_func_t *) scan_timer_cb, (void *) 0);
	os_timer_arm(&scan_timer, 100, true);//100ms je Kanal

	//SW-Timer - Status nach Interval initialisieren
	os_timer_disarm(&statusTx_interval);
	os_timer_setfn(&statusTx_interval, (os_timer_func_t *) statusTx_interval_cb, (void *) 0);
	os_timer_arm(&statusTx_interval, interval_ms, TRUE);
	//Task - Staus senden nach Interval oder Schaltbefehl
	system_os_task(send_actor_status_task, statusTx_TaskPrio, statusTx_TaskQueue, statusTx_TaskQueueLen);
}

void user_rf_pre_init(void) {

//	WRITE_PERI_REG(0x600011f4, 1 << 16 | 11);
//	wifi_set_phy_mode(PHY_MODE_11B);
}

// *** CRC16 ****************************************************************************

#define POLY 0x8408
/*
 //                                      16   12   5
 // this is the CCITT CRC 16 polynomial X  + X  + X  + 1.
 // This works out to be 0x1021, but the way the algorithm works
 // lets us use 0x8408 (the reverse of the bit pattern).  The high
 // bit is always assumed to be set, thus we only use 16 bits to
 // represent the 17 bit value.
 */

int16_t IROM crc16(char *data_p, uint16_t length) {
	unsigned char i;
	unsigned int data;
	unsigned int crc = 0xffff;

	if (length == 0) return (~crc);

	do {
		for (i = 0, data = (unsigned int) 0xff & *data_p++; i < 8; i++, data >>= 1) {
			if ((crc & 0x0001) ^ (data & 0x0001)) crc = (crc >> 1) ^ POLY;
			else crc >>= 1;
		}
	} while (--length);

	crc = ~crc;
	data = crc;
	crc = (crc << 8) | (data >> 8 & 0xff);

	return (crc);
}

//---------------------------------------------------------------------------------------------------------------
//System

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
 *******************************************************************************/
uint32 IROM user_rf_cal_sector_set(void) {
	enum flash_size_map size_map = system_get_flash_size_map();
	uint32 rf_cal_sec = 0;

	switch (size_map) {
	case FLASH_SIZE_4M_MAP_256_256:
		rf_cal_sec = 128 - 5;
		break;

	case FLASH_SIZE_8M_MAP_512_512:
		rf_cal_sec = 256 - 5;
		break;

	case FLASH_SIZE_16M_MAP_512_512:
	case FLASH_SIZE_16M_MAP_1024_1024:
		rf_cal_sec = 512 - 5;
		break;

	case FLASH_SIZE_32M_MAP_512_512:
	case FLASH_SIZE_32M_MAP_1024_1024:
		rf_cal_sec = 1024 - 5;
		break;

	case FLASH_SIZE_64M_MAP_1024_1024:
		rf_cal_sec = 2048 - 5;
		break;
	case FLASH_SIZE_128M_MAP_1024_1024:
		rf_cal_sec = 4096 - 5;
		break;
	default:
		rf_cal_sec = 0;
		break;
	}
	return rf_cal_sec;
}

// Time (Debug)------------------------------------------------------------

int now()
{
	return system_get_time() / 1000;
}

void ptime(char *text)
{
	#ifdef DEBUG_X
		os_printf("[%04d] %s", now(), text);
	#endif
}

//**************************************************************************************

