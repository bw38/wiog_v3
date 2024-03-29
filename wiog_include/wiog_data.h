/*
 * wiog_data.h
 *
 *  Created on: 21.05.2020, Author: DL7VMD
 *
 *  WIOG - Wireless Internet of Garden
 *  Datensätze device - Gateway
 *
 */

#ifndef __WIOG_DATA_H__
#define __WIOG_DATA_H__

#include "wiog_system.h"

#define MAX_DATA_LEN 1024				//maximale Payload-Datenlänge

/*
//Standard-Frame-Typ
typedef struct __attribute__((packed)) {
	uint8_t  datatype;
	uint8_t  index;
	uint8_t  status;	//0 = Ok / !=0 -> Fehler
	uint8_t val_U8A;
	int32_t val_I32A;
	int32_t val_I32B;
	int32_t val_I32C;
} data_entry_t;
*/
// --------------------------------------------------

// synchron halten mit mqtt.cfg
typedef enum {
	dt_dummy 	= 0,
	dt_bme280   = 1,
	dt_bme680   = 2,
	dt_level    = 3,
	dt_ds18b20  = 4,
	dt_heatctrl = 5,
	dt_txt_info = 6,
	dt_ubat_mv  = 7,
	dt_am2302   = 8,
	dt_bitmask  = 9,
	dt_timer_sek= 10,
	dt_sht3x	= 11,
	dt_ns_sw	= 12,
	dt_res_err  = 13,

	dt_runtime_ms = 21,
	dt_cycle   = 22,

	//Systemdaten werden nicht an MQTT-Server weitergeleitet
	dt_snr_info = 101,
	dt_unknown = 255
} datatype_t;


typedef enum {
	DF_NULL= 0,
	DF_I32 = 1,
	DF_I64 = 2,
	DF_STR = 3
} data_frame_t;

typedef struct __attribute__((packed)) {
	uint8_t  frametype;	//const I32
	uint8_t  datatype;
	uint8_t  index;
	int8_t   status;	//0 = Ok / !=0 -> Fehler
	int32_t  value;
} df_i32_t;

typedef struct __attribute__((packed)) {
	uint8_t  frametype;	//const I64
	uint8_t  datatype;
	uint8_t  index;
	int8_t   status;	//0 = Ok / !=0 -> Fehler
	int64_t  value;
} df_i64_t;

typedef struct __attribute__((packed)) {
	uint8_t frametype;	//const STR
	uint8_t datatype;
	uint8_t index;
	uint8_t length;
	uint8_t txt[];
} df_str_t;




// --------------------------------------------------------

//#define MANAGEMENT_SZ 32
typedef struct __attribute__((packed)) {
	uint16_t crc16;			//Prüfsumme ab len-word
	uint16_t len;
	uint16_t uid;			//ID Sensor/Actor, crc16 aus efuse-mac
	uint8_t  wifi_channel;	//Arbeitskanal
	uint8_t  species;
	int8_t   tx_pwr;
	uint8_t  cnt_entries;	//Anzahl der Datensätze
	uint32_t cycles;
	uint32_t ulp_cycles;
	uint32_t onTime;		//sensor - ms / actor - sek
	uint32_t sz_heap;		//aktuelle Größe Heap, Debuging

	union {
		struct {					// to Dev -> GW
			uint32_t cnt_no_response;	//Anzahl der nicht übermittelten Datenpakete
			uint32_t cnt_tx_repeat;	//Anzahl der Tx-Wiederholungen
		};
		int64_t utime; 				//from GW -> Actor
	};
} management_t;



typedef struct __attribute__((packed)) {
	management_t man;
	uint8_t data[1024];	//max Datenlänge !!!
	uint32_t ix;
} payload_t;




extern void add_entry_I32 (payload_t* pl, uint8_t type, uint8_t ix, uint8_t st, int32_t val);
extern void add_entry_I64 (payload_t* pl, uint8_t type, uint8_t ix, uint8_t st, int64_t val);
extern void add_entry_buf (payload_t* pl, uint8_t type, uint8_t ix, uint8_t len, uint8_t* buf);
extern void add_entry_str (payload_t* pl, uint8_t type, uint8_t ix, char* str);
extern void add_entry_snr (payload_t* pl, dev_uid_t uid, uint8_t snr);
extern void* get_next_entry (payload_t* pl, data_frame_t* dft);



// -----------------------------------------------------------------------
//Mesh
//Prio-Liste zur Entscheidung welcher Repeater oder Gateway auf das Datenpaket eines Device-UID antwortet

// Node-Information-Block NIB
// Der NIB wird vom GW erzeugt und als Broadcast gesendet, incl Localtime
// Die Nodes empfangen den NIB und aktualisieren ggf den lokalen NIB im DRAM
// Nodes senden ihren eigenen NIB zur Synchronisation anderer Nodes

/*
typedef struct __attribute__((packed)) {
	dev_uid_t node_uid;
	uint8_t snr;
} node_info_t;
*/

typedef struct __attribute__((packed)) {
	dev_uid_t dev_uid;				//0 -> ungültig / uint16_t -> gültig
	int8_t	  best_snr;				//höchste SNR GW oder Node
	dev_uid_t node_uid[2];		 	//Node-Priorität	UID/SNR
} dev_info_line_t;

// Node-Info-Block NIB - wird in Raspi-GW erstellt und per Broacast an Nodes verteilt
typedef struct __attribute__((packed)) {
	int64_t ts;							//Timestamp zur Aktualitätsprüfung
	uint8_t bc_interval_sek;
	uint8_t dev_cnt;					//Anzahl der Einträge in dev_info[]
	dev_uid_t slot_info[MAX_SLOTS];		//Timeslots der Nodes (UID)
	dev_info_line_t dev_info[MAX_DEVICES]; 	//Eintrag je Device, DevUID: NodeUID/SNR -> (sortiert nach snr am Node)
} node_info_block_t;

int nib_get_uid_ix(node_info_block_t *pnib, dev_uid_t uid);
int nib_get_node_slot(node_info_block_t *pnib, dev_uid_t uid);
int nib_get_priority(node_info_block_t *pnib, dev_uid_t dev_uid, dev_uid_t node_uid);
int nib_get_size(node_info_block_t* pnib);
int8_t nib_get_best_snr(node_info_block_t *pnib, dev_uid_t dev_uid);
void nib_clear_all(node_info_block_t *pnib);


// Node-Device_Info
// Einzel-Info eines Nodes zum GW: Dev_UID / SNR
// GW-RPI sammelt die Infos und estellt zyklisch den NIB
typedef struct __attribute__((packed)) {
	dev_uid_t my_uid;
	dev_uid_t dev_uid;
	uint8_t   snr;
} node_dev_info_t;


// ---------------------------------------------------------------------

// Device-Info-Block
// Geräte-Informationen RPi-GW -> ESP-GW -> Actor/Sensor
//Einzelinformation
typedef struct __attribute__((packed)) {
	dev_uid_t uid;
	uint8_t  species;
	uint32_t interval_ms;
	uint32_t return_val;   //gerätespezifischer Rückgabewert
	uint8_t  min_snr_db;
} device_info_t;

//Datenstruktur wird im RPi - TDeviceInfoBlock.Data2Buffer() zusammengestellt
typedef struct __attribute__((packed)) {
	uint32_t	  def_sensor_interval_ms;
	uint32_t	  def_actor_interval_ms;
	uint32_t	  def_node_interval_ms;
	device_info_t device_info[MAX_DEVICES-1];
} device_info_block_t;


// ---------------------------------------------------------------------

#endif  //__WIOG_DATA_H__
