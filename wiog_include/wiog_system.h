#ifndef __WIOG_SYSTEM_H__
#define __WIOG_SYSTEM_H__

/*
 *  Internet of Garden
 *  ==================
 *  gemeinsame Definitionen ab Version 3.0
 *
 *  Header-File wird über das Dateisystem in den main-folder der Projekte verlinkt (ln)
 *  PREFIX "ln_" -> Eintrag in seafile-ignore.txt
 *
 */


#define DEBUG_X


#include "esp_wifi.h"

#define MS 1/portTICK_PERIOD_MS	//1 Tick je 10ms

#define MAX_DEVICES	64			//Max n Devices lassen sich im Netzwerk verwalten (s. Meshliste)
#define MAX_NODES	4			//Max n-1 Nodes + 1GW lassen sich verwalten
#define MAX_SLOTS	4			//Anzahl der Node-Time-Slots

//Wertebereich esp_wifi_set_max_tx_power(x)
#ifdef DEBUG_X	//reduzierte Leistung im Debug-Modus
#define MAX_TX_POWER 84 //20  	// => 5dBm
#define MIN_TX_POWER 8		// => 2dBm
#else
#define MAX_TX_POWER 84  	// => 20dBm
#define MIN_TX_POWER 8		// => 2dBm
#endif
#define IDEAL_SNR 12		//(dB) Sensoren einpegeln

#define SEK	1e3		//ms -> Sek
#define MIN 60e3	//ms -> Min
#define STD 60*MIN	//ms -> Std

//#define FW_VERSION				03
//#define FW_REVISION				00
//#define MIN_SLEEP_TIME_MS		3*SEK
//#define MAX_SLEEP_TIME_MS		12*STD
//#define DEFAULT_SLEEP_TIME_MS 10*MIN
//#define MY_SPECIES			SPECIES_SENSOR
#define FIRST_SLEEP_MS			3*SEK

#define SLOT_TIME_US 			8000		//Repeater und Gateway - n ms
#define TX_REPEAT_INTERVAL		60*MS		//Tx-Wiederholung bei fehlenden ACK
#define TX_REPEAT_CNT_MAX		5

#define SENSOR_MIN_SLEEP_TIME_MS  10*SEK
#define SENSOR_DEF_SLEEP_TIME_MS  10*MIN
#define SENSOR_MAX_SLEEP_TIME_MS  24*STD

#define ACTOR_MIN_SLEEP_TIME_MS 5*1000		//min 5Sek
#define ACTOR_MAX_SLEEP_TIME_MS 60*60*1000	//max 1h
#define ACTOR_DEF_SLEEP_TIME_MS 30*1000		//default 30sek


#define AES_KEY		0xf4, 0x45, 0xfa, 0xf7, 0x5b, 0x5a, 0x44, 0x53,\
			 	 	0x01, 0x1a, 0x4b, 0xb2, 0xdd, 0x45, 0x78, 0xda

#define AES_IV 		0xf4, 0x15, 0x0d, 0x30, 0x4f, 0x00, 0xe1, 0x80,\
					0x98, 0x49, 0x1a, 0xf9, 0x59, 0x49, 0x68, 0x49


typedef uint8_t mac_addr_t[6];

static const wifi_country_t wifi_country_de = {
		.cc="DE",
		.schan=1,
		.nchan=13,
		.max_tx_power=MAX_TX_POWER,
		.policy=WIFI_COUNTRY_POLICY_AUTO
};

#define MAC_BYTE5 0xc4    // Testumgebung    (testgw)
//#define MAC_BYTE5 0xc8		// Arbeitsumgebung (raspigw)

//Netzwerk-Kennung
//LSB in Byte1 muss 1 sein, sonst unkrontrollierte Paketwiederholungen esp_wifi_80211_tx (nur esp8266 rtos)
static const mac_addr_t mac_net =      { 0xf9, 0xfe, 0x36, 0x96, MAC_BYTE5, 0x00 };
static const mac_addr_t mac_gateway  = { 0xf9, 0xfe, 0x36, 0x96, MAC_BYTE5, 0x01 };
static const mac_addr_t mac_sensor   = { 0xf9, 0xfe, 0x36, 0x96, MAC_BYTE5, 0x02 };	//Byte 6 wird  individuell ersetzt
static const mac_addr_t mac_actor    = { 0xf9, 0xfe, 0x36, 0x96, MAC_BYTE5, 0x03 };
static const mac_addr_t mac_repeater = { 0xf9, 0xfe, 0x36, 0x96, MAC_BYTE5, 0x04 };


typedef uint16_t dev_uid_t;	//16 Bit Geräte-UID -> Gefahr der ID-Dopplung !

#define GW_UID 0xFFFF


//Wakeup-Quellen der Sensoren
typedef enum {
	WS_ULP,
	WS_MAIN,
}wakeup_src_t;


//entspricht dem letzten Byte der MAC-Adresse (mac5)
typedef enum {
	DUMMY = 0,
	GATEWAY = 1,
	SENSOR = 2,
	ACTOR = 3,
	REPEATER = 4,
} species_t;

//esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);




//Vendor-Frame-Type
typedef enum {
	UNKNOWN = 0,
    SCAN_FOR_CHANNEL,	//Device sucht Kanal
	ACK_FOR_CHANNEL,	//Repeater und GW antworten auf Kanalsuche
	DATA_TO_GW,			//DataFrame Device --> Gateway
	ACK_FROM_GW,		//Empfangsbestätigung des GW
	DATA_FROM_GW,
	ACK_TO_GW,
	BC_NIB,				//Boradcast Node Info Block
	RESET_DEV			//Reset	GW -> Device
} vtype_t;


typedef struct __attribute__((packed)){
	//MAC-Header Byte 00 .. 23
	uint16_t frame_ctrl;
	uint16_t duration;
	mac_addr_t mac_to;
	mac_addr_t mac_from;
	mac_addr_t mac_net;
	uint16_t seq_ctrl;
	//Vendor-Header 24..n
	uint8_t vtype;
	uint8_t species;
	int8_t txpwr;
	uint8_t channel;
	uint16_t uid;
	uint32_t frameid;			//Frame-ID f. verify Ack, Random32
	uint32_t interval_ms;
	union {	//Status infos (Repeater-Infos), derzeit keine betriebswichtigen Daten
		struct { uint8_t tagA; uint8_t tagB; int16_t tagC; };
		uint32_t tagD;
	};
	union {	//32bit Rückgabe Device-Infos GW -> actor -> sensor, bspw Thresholds
		struct { uint8_t rdi8A; uint8_t rdi8B; uint8_t rdi8C; uint8_t rdi8D;};
		struct { uint16_t rdi16A; uint16_t rdi16B;};
		uint32_t rdi32;
	};
	//letzter Eintrag
	uint16_t hdr_sign;	//Header-Signature CRC16
} wiog_header_t;


static const wiog_header_t dummy_header = {
	.frame_ctrl = 0x00d0,	//Action-Frame
	.duration = 0,
	.seq_ctrl = 0,
	.vtype = UNKNOWN,
	.species = DUMMY,
	.txpwr = 0,
	.channel = 0,
	.uid = 0,
	.frameid = 0,
	.tagD = 0,
	.rdi32 = 0
};


//Payload (Wifi Rx)
typedef struct __attribute__((packed)){
	wiog_header_t header;
	uint8_t pdata[];
} wiog_data_frame_t;


//WIOG Queue-Handling
// ----------------------

typedef struct __attribute__((packed)){ //cb stellt Rx-Daten in die Rx-Queue
	wifi_pkt_rx_ctrl_t rx_ctrl;
	wiog_header_t wiog_hdr;
	int64_t timestamp;
	int data_len;
	uint8_t *pdata;
} wiog_event_rxdata_t;

typedef struct __attribute__((packed)){ //Tx-Daten in die Tx-Queue stellen
	wiog_header_t wiog_hdr;
	int64_t target_time;	//Sendezeitpunkt -> obsolete !!!
	uint8_t tx_max_repeat;	//Tx-Wiederholung 0 => es wird kein Ack erwartet
	bool crypt_data;		//true -> Datenblock wird verschlüsselt
	esp_timer_handle_t h_timer;
	uint16_t data_len;		//Länge des Datenpaketes
	uint8_t  *pdata;		//Pointer auf Datenpaket
} wiog_event_txdata_t;


// UART - Handling
// --------------------------
typedef struct __attribute__((packed)){
	uint16_t  data_len;
	uint8_t   data[1500];
} uart_payload_t;

// -----------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------

esp_err_t event_handler(void *ctx, system_event_t *event);

void tstart(uint8_t ix );
int tstop(uint8_t ix);

void hexdump(uint8_t *data, int len);
void hexdumpex(uint8_t *data, int len);

int get_blocksize(int data_len);
int cbc_encrypt(uint8_t *data, uint8_t *crypted, int data_len, uint8_t *key, int key_len);
int cbc_decrypt(uint8_t *crypted, uint8_t *data, int len, uint8_t *key, int key_len);
int wiog_encrypt_data(uint8_t* data, uint8_t* encrypted, uint16_t len, uint32_t fid);
int wiog_decrypt_data(uint8_t* encrypted, uint8_t*data, uint16_t len, uint32_t fid);


void wiog_set_channel(uint8_t ch);

uint16_t get_uid();
wiog_header_t wiog_get_dummy_header(uint8_t mac_to, uint8_t mac_from);

uint16_t crc16(const uint8_t *data_p, uint16_t length);

//NVS
esp_err_t init_nvs();
uint8_t nvs_get_wifi_channel();
void nvs_set_wifi_channel(uint8_t ch);
int32_t nvs_get_sysvar(uint8_t ix);
void nvs_set_sysvar(uint8_t ix, int32_t value);
int now();
extern int64_t tStart;

//Debug
#ifdef DEBUG_X
	void tstart(uint8_t ix );
	int tstop(uint8_t ix);

	void compare_set_get_tx_power();
#endif



#endif	// __IOG_H__
