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
#include "../../wiog_include/wiog_wifi_actor.h"

#define VERSION  3
#define REVISION 0

#undef  LOG_LOCAL_LEVEL	//Warnhinweis vermeiden
#define LOG_LOCAL_LEVEL ESP_LOG_NONE	//s. readme.txt
//zusätzlich Bootloader-msg  mit GPIO_15 -> low unterdrücken

//Prototypes
void ShowDateTime(time_t dt);

// ----------------------------------------------------------------------------------------------

//CallBack des Wiog-Headers nach Rx ACK, auch nach SNR-Data-Frame
void rx_ack_handler(wiog_header_t* pHdr) {
//	LED_STATUS_OFF;
	#ifdef DEBUG_X
		printf("[%04d]Rx-ACK\n", now());
	#endif
}

//
void rx_data_handler(wiog_header_t* pHdr, payload_t* pl, int len)  {
hexdump((uint8_t*)pl, len);
ShowDateTime(pl->man.utime);

	pl->ix = 0;
	data_frame_t dft;
	for (int i=0; i<pl->man.cnt_entries; i++) {
		void* entry = get_next_entry(pl, &dft);
		switch (dft) {
		case DF_I32:
			printf("I32\n");
			break;

		case DF_I64:
			printf("I64\n");
			break;
		case DF_STR:
			printf("STR\n");
			break;
		case DF_NULL:
			break;
		}
	}


	#ifdef DEBUG_X
		printf("[%04d]Rx-Data\n", now());
	#endif
}

// ----------------------------------------------------------------------------------------------



void app_main(void) {
	//Initialisierung --------------------------------------------------------------------------------
	version = VERSION;
	revision = REVISION;

	cb_rx_ack_handler = &rx_ack_handler;
	cb_rx_data_handler = &rx_data_handler;

	nvs_flash_init();

    set_species(ACTOR);	//Repeater-Funktion kann in Ack-Frame zugewiesen werden (DIB im GW)

    wiog_wifi_actor_init();
	printf("Actor-UID: %d\n", my_uid);

	// Main-Loop - Statusmeldung
	while (true) {
		//falls zuvor auf 2 Dataframes das ACK ausgebleiben ist -> Channel-Scan veranlassen
		if (cnt_no_response_serie > 1) wifi_channel = 0;

		//ggf Channel-Scan veranlassen
		if (wifi_channel == 0) wiog_set_channel(0);
		else {
			//Test-Frame senden ---------------------------------------------
			char txt[] = {"Hello World - How are you ? Das ist ein Test"};

			uint8_t sz = strlen(txt) & 0xFF;
			uint8_t data[sz+1];				//Byte 0 => Längenbyte
			memcpy(&data[1], txt, sz);		//Byte 1 => Datenbereich
			data[0] = sz;

			payload_t pl;
			pl.ix = 0;
			set_management_data(&pl.man);
			add_entry_str (&pl, dt_txt_info, 1, txt);
			//Data to GW
			send_data_frame(&pl, pl.ix + sizeof(pl.man), ACTOR);

			vTaskDelay(interval_ms / portTICK_PERIOD_MS);
		}

	}	//While
}

// -----------------------------------------------------------------------------------


void ShowDateTime(time_t dt) {
	char buf[32]; //= "tTime.txt=\"";
	struct tm timeinfo;
	localtime_r(&dt, &timeinfo);
	strftime(buf, 20, "%Y-%m-%d %H:%M:%S", localtime(&dt));
	printf("Time: %s\n", buf);
}


