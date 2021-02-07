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

// --------------------------------------------------

//veraltet
typedef enum {
	DF_I32 = 0,
	DF_I64 = 1,
	DF_STR = 2
}data_frame_t;

typedef struct __attribute__((packed)) {
	uint8_t  frametype;	//const I32
	uint8_t  datatype;
	uint8_t  index;
	uint8_t  status;	//0 = Ok / !=0 -> Fehler
	int32_t value;
} df_i32_t;

typedef struct __attribute__((packed)) {
	uint8_t  frametype;	//const I64
	uint8_t  datatype;
	uint8_t  index;
	uint8_t  status;	//0 = Ok / !=0 -> Fehler
	int64_t value;
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
	uint32_t sid;			//Systemkennung zur Validierung der Daten nach Decrypt (21041959)
	uint16_t uid;			//ID Sensor/Actor, crc16 aus efuse-mac
	uint8_t  wifi_channel;	//Arbeitskanal
	uint8_t  version;
	uint8_t  revision;
	uint8_t  species;
	uint8_t  rssi;
	uint8_t  tx_pwr;
	uint32_t cycle;
	uint32_t interval;
	uint64_t localtime;
	uint16_t cnt_no_response;
	uint8_t  cnt_entries;	//Anzahl der Datensätze
	uint8_t  res8A;
} management_t;



typedef struct __attribute__((packed)) {
	management_t man;
	uint8_t data[256];	//max Datenlänge !!!
	uint32_t ix;
} payload_t;


// -----------------------------------------------------------------------
//Mesh
//Prio-Liste zur Entscheidung welcher Repeater oder Gateway auf das Datenpaket eines Device-UID antwortet
//Daten f. Response-Frame
typedef struct __attribute__((packed)) {
	int32_t  uid;			//-1 -> ungültig / uint16_t -> gültig
	uint32_t interval_ms;	//Device-Interval für Antwort-Frame
	uint8_t  priority;		//f. Time-Slot -> ACK to Device
	uint8_t	 standby;		// != 0 -> Device soll auf Anweisung des Gateway warten
	int64_t  ts;			//Zeitpunkt - zuletzt gesetzt (f. cleaning)
}mesh_devices_t[MAX_DEVICES];



//------------------------------------------------------------------------



extern void add_entry_I32 (payload_t* pl, uint8_t type, uint8_t ix, uint32_t st, int32_t val);
extern void add_entry_I64 (payload_t* pl, uint8_t type, uint8_t ix, uint32_t st, int64_t val);
extern void add_entry_buf (payload_t* pl, uint8_t type, uint8_t ix, uint8_t len, uint8_t* buf);
extern void add_entry_str (payload_t* pl, uint8_t type, uint8_t ix, char* str);
extern void* get_next_entry (payload_t* pl, data_frame_t* dft);


//Mesh - Prio -Liste
int mesh_get_uid_ix(mesh_devices_t mds, int32_t uid);
int32_t mesh_get_priority(mesh_devices_t mesh_list, uint16_t uid);
void mesh_set_priority(mesh_devices_t mesh_list, uint16_t uid, int32_t prio);
void mesh_clear_list(mesh_devices_t mesh_list);
void mesh_clear_all(mesh_devices_t mesh_list);




#endif  //__WIOG_DATA_H__
