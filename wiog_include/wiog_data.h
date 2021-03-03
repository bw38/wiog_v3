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

typedef enum {
	DF_I32 = 0,
	DF_I64 = 1,
	DF_STR = 2,
	DF_SNR = 3,
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

// Node -> GW
typedef struct __attribute__((packed)) {
	uint8_t   frametype;	//const DEV_SNR
	dev_uid_t dev_uid;
	uint8_t   snr;
} df_snr_t;

static const df_snr_t dev_snr_0 = {
	.frametype = DF_SNR,
};

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
	uint8_t *data;	//max Datenlänge !!!
	uint32_t ix;
} payload_t;




extern void add_entry_I32 (payload_t* pl, uint8_t type, uint8_t ix, uint32_t st, int32_t val);
extern void add_entry_I64 (payload_t* pl, uint8_t type, uint8_t ix, uint32_t st, int64_t val);
extern void add_entry_buf (payload_t* pl, uint8_t type, uint8_t ix, uint8_t len, uint8_t* buf);
extern void add_entry_str (payload_t* pl, uint8_t type, uint8_t ix, char* str);
extern void* get_next_entry (payload_t* pl, data_frame_t* dft);



// -----------------------------------------------------------------------
//Mesh
//Prio-Liste zur Entscheidung welcher Repeater oder Gateway auf das Datenpaket eines Device-UID antwortet

// Node-Information-Block NIB
// Der NIB wird vom GW erzeugt und als Broadcast gesendet, incl Localtime
// Die Nodes empfangen den NIB und aktualisieren ggf den lokalen NIB im DRAM
// Nodes senden ihren eigenen NIB zur Synchronisation anderer Nodes

typedef struct __attribute__((packed)) {
	dev_uid_t dev_uid;					//0 -> ungültig / uint16_t -> gültig
	dev_uid_t node_uids[MAX_NODES]; 	//Node-Priorität
} dev_info_t;

// Node-Info-Block NIB	(888 Bytes)
typedef struct __attribute__((packed)) {
	int64_t ts;							//Timestamp zur Aktualitätsprüfung
	dev_uid_t slot_info[MAX_SLOTS];
	dev_info_t dev_info[MAX_DEVICES]; 	//Position im Array entspricht Routing-Prio
} node_info_block_t;

int nib_get_uid_ix(node_info_block_t *pnib, dev_uid_t uid);
int nib_get_priority(node_info_block_t *pnib, dev_uid_t dev_uid, dev_uid_t node_uid);
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

#endif  //__WIOG_DATA_H__
