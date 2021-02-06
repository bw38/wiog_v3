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
#include "../../wiog_include/wiog_actor.h"

#define VERSION  3
#define REVISION 0

payload_t tx_payload;
payload_t rx_payload;
int ixtxpl = 0;

// ---------------------------------------------------------------------------------

//Verarbeitung der empfangenen GW-Daten
void rx_data_cb (uint8_t* payload) {
hexdump(payload, 64);
	payload_t* ppl = (payload_t*) payload;
	ppl->ix = 0;	//Init Listenanfang

	for (uint8_t i = 0; i < ppl->man.cnt_entries; i++) {
		data_frame_t dft = -1;
		void* pEnt = get_next_entry(ppl, &dft);

		if (dft == DF_I32) {
			df_i32_t*  pE32 = pEnt;
			printf("I32: typ: %d | ix:%d | st:%d | val:%d\n", pE32->datatype, pE32->index, pE32->status, pE32->value);
		}
		if (dft == DF_I64) {
			df_i64_t*  pE64 = pEnt;
			printf("I64: typ: %d | ix:%d | st:%d | val:%d\n", (int)pE64->datatype, (int)pE64->index, (int)pE64->status, (int)pE64->value);
		}
		if (dft == DF_STR) {
			df_str_t*  pEstr = pEnt;
			printf("STR: typ: %d | ix:%d | len:%d | txt:%s\n", pEstr->datatype, pEstr->index, pEstr->length, pEstr->txt);

		}
	}

}


//Eintragen der Management-Daten in den Payload
//Aufrufer aktualisiert später die Anzahl der Datensätze
void set_management_data (management_t* pMan) {
	pMan->sid = SYSTEM_ID;
	pMan->uid = dev_uid;
	pMan->wifi_channel = wifi_channel;
	pMan->version = VERSION;
	pMan->revision = REVISION;
	pMan->cycle = cycle;
	pMan->cnt_no_response = cnt_no_response;
	int8_t pwr;
	esp_wifi_get_max_tx_power(&pwr);
	pMan->cnt_entries = 0;
	ixtxpl = 0;
}


void app_main(void) {

	wiog_actor_init();
	wiog_rx_register_cb(rx_data_cb);
	printf("Actor-UID: %d\n", dev_uid);

	while (true) {
		//ggf Channel-Scan vweranlassen
		if (wifi_channel == 0) wiog_set_channel(0);

		char txt[] = {"Hello World - I'm an actor !"};

		uint8_t sz = strlen(txt) & 0xFF;
		uint8_t data[sz+1];				//Byte 0 => Längenbyte
		memcpy(&data[1], txt, sz);		//Byte 1 => Datenbereich
		data[0] = sz;

//		send_data_frame(data);

		vTaskDelay(interval_ms / portTICK_PERIOD_MS);

	}	//While
}

// -----------------------------------------------------------------------------------

