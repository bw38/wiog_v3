/*
    Datensätze zur Kommunikation Device - Gateway
*/
#include "string.h"

#include "wiog_data.h"


void add_entry_I32 (payload_t* pl, uint8_t type, uint8_t ix, uint32_t st, int32_t val) {
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


void add_entry_I64 (payload_t* pl, uint8_t type, uint8_t ix, uint32_t st, int64_t val) {
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
			pl->ix += sizeof(df_str_t) /*- sizeof(ent->txt) */+ ent->length;
			break;
		}
		default: entry = NULL;
	}

	return entry;
}
