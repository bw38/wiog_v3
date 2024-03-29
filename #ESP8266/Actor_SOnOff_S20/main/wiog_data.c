/*
    Datensätze zur Kommunikation Device - Gateway
*/
#include "string.h"
#include "wiog_data.h"

/* Standard-Frame-Typ ???
void add_entry(payload_t* ppl, data_entry_t* pdent ){
	memcpy(&ppl->data[ppl->ix], pdent, sizeof(data_entry_t));
	ppl->ix += sizeof(data_entry_t);
	ppl->man.cnt_entries++;
}
*/

void add_entry_I32 (payload_t* pl, uint8_t type, uint8_t ix, uint8_t st, int32_t val) {
	df_i32_t e32 = {
		.frametype = DF_I32,
		.datatype = type,
		.index = ix,
		.status = st,
		.value = val
	};
	memcpy(&pl->data[pl->ix], &e32, sizeof(df_i32_t));
	pl->ix += sizeof(df_i32_t);
	pl->man.cnt_entries++;
}


void add_entry_I64 (payload_t* pl, uint8_t type, uint8_t ix, uint8_t st, int64_t val) {
	df_i64_t e64 = {
		.frametype = DF_I64,
		.datatype = type,
		.index = ix,
		.status = st,
		.value = val
	};
	memcpy(&pl->data[pl->ix], &e64, sizeof(df_i64_t));
	pl->ix += sizeof(df_i64_t);
	pl->man.cnt_entries++;
}

//Byte-Array mit Längenangabe
void add_entry_buf (payload_t* pl, uint8_t type, uint8_t ix, uint8_t len, uint8_t* buf) {
	df_str_t ent = {
		.frametype = DF_STR,
		.datatype = type,
		.index = ix
	};
	ent.length = len;
	uint8_t sz = sizeof(df_str_t);// - sizeof(ent.txt); //Size der Verwaltungsdaten des Eintrages
	memcpy(&pl->data[pl->ix], &ent, sz);	//Verwaltungsdaten kopieren
	pl->ix += sz;
	memcpy(&pl->data[pl->ix], buf, ent.length);
	pl->ix += ent.length;
	pl->man.cnt_entries++;
}


//Null-Terminierter String
void add_entry_str (payload_t* pl, uint8_t type, uint8_t ix, char* str) {
	df_str_t ent = {
		.frametype = DF_STR,
		.datatype = type,
		.index = ix
	};
	ent.length = strlen(str) + 1;	//incl. Null-Terminierung
	uint8_t sz = sizeof(df_str_t);// - sizeof(ent.txt); //Size der Verwaltungsdaten des Eintrages
	memcpy(&pl->data[pl->ix], &ent, sz);
	pl->ix += sz;
	memcpy(&pl->data[pl->ix], str, ent.length);
	pl->ix += ent.length;
	pl->man.cnt_entries++;
}



// -------------------------------------------------------------------------------

//liefert untypisierten Zeiger auf DataEntry und DataFrameTyp
void* get_next_entry (payload_t* pl, data_frame_t* dft) {
	*dft = (data_frame_t) pl->data[pl->ix];
	void* entry = &pl->data[pl->ix];

	switch (*dft) {
		case DF_I32: pl->ix += sizeof(df_i32_t); break;
		case DF_I64: pl->ix += sizeof(df_i64_t); break;
		case DF_STR: {
			df_str_t* ent = entry;
			pl->ix += sizeof(df_str_t) + ent->length;
			break;
		}
		default: entry = NULL;
	}

	return entry;
}

// -----------------------------------------------------------------------------

//Node-INfo_Block NIB - Prio-Node-Liste f. Repeater

//Listenplatz im NodeInfoBlock einer uid suchen
//bei uid == 0 -> ersten freien Platz zurückliefern
//Result: 0..n / -1 -> uid nicht gefunden
int nib_get_uid_ix(node_info_block_t *pnib, dev_uid_t uid){
	int res = -1;
	for (int i=0; i< MAX_DEVICES; i++)
		if (pnib->dev_info[i].dev_uid == uid){
			res = i;
			break;
		}
	return res;
}

//Slot eines Nodes f. Channelscan ermitteln
//auch f. unbekannte Dev-UID
//1 -> höchte Prio
int nib_get_node_slot(node_info_block_t *pnib, dev_uid_t uid) {
	int res = MAX_SLOTS;	//wenn nicht gefunden -> niedrige Prio
	for (int i=0; i<MAX_SLOTS; i++) {
		if (pnib->slot_info[i] == uid) {
			res = i++;
			break;
		}
	}
	return res;
}

//Priorität eines Knotens (node_uid) für Kommunikation mit Device (dev_uid)
//höchste Prio = 1
//GW und Null-Einträge werden nicht mitgezählt
int nib_get_priority(node_info_block_t *pnib, dev_uid_t dev_uid, dev_uid_t node_uid ) {
	int res = 1;
	int ix = nib_get_uid_ix(pnib, dev_uid);
	if (ix >= 0) {
		for (int i = 0; i < 2; i++) {	//2 Nodes je device im NIB
			dev_uid_t uid = pnib->dev_info[ix].node_uid[i];
			if (uid == node_uid) return res;
			res++;
		}
	} else {	//Dev-UID nicht gefunden -> Node in Slot-Liste suchen
		return nib_get_node_slot(pnib, node_uid);
	}
	return MAX_SLOTS;	//nicht gefunden
}

//dyn. Größe des NIB in Bytes, abhängig von der Anzahl der Device-Einträge
int nib_get_size(node_info_block_t* pnib) {
	return
		sizeof(int64_t) + 	//ts
		sizeof(uint8_t) +	//bc_interval_sek
		sizeof(uint16_t) +	//dev_cnt
		sizeof(dev_uid_t) * MAX_SLOTS +
		sizeof(dev_info_line_t) * pnib->dev_cnt;
}



//SNR des ersten einer UID zugeordneten Nodes/GW ermitteln
int8_t nib_get_best_snr(node_info_block_t *pnib, dev_uid_t dev_uid) {
	int ix = nib_get_uid_ix(pnib, dev_uid);
	if (ix >= 0)	//falls Geräteeintrag vorhanden
		return pnib->dev_info[ix].best_snr;
	else
		return 0;
}



//NIB initialisieren
void nib_clear_all(node_info_block_t *pnib) {
	bzero(pnib, sizeof(node_info_block_t));
}
