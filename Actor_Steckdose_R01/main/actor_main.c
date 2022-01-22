#include "freertos/FreeRTOS.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#include "nvs_flash.h"
#include "string.h"

#include "wiog_include/wiog_system.h"
#include "wiog_include/wiog_data.h"
#include "wiog_include/wiog_wifi_actor.h"

#include "interface.h"
#include "ns_socket.h"

#define VERSION  3
#define REVISION 0

#undef  LOG_LOCAL_LEVEL	//Warnhinweis vermeiden
#define LOG_LOCAL_LEVEL ESP_LOG_NONE	//s. readme.txt
//zusätzlich Bootloader-msg  mit GPIO_15 -> low unterdrücken

#define MEASURE_QUEUE_SIZE 32
xQueueHandle measure_response_queue;	//Flags -> Mess-Ereigni

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
	#ifdef DEBUG_X
		hexdump((uint8_t*)pl, len);
		ShowDateTime(pl->man.utime);
	#endif
	df_i32_t* pi32;
//	df_i64_t* pi64;
//	df_str_t* pstr;
	pl->ix = 0;
	data_frame_t dft;

	for (int i=0; i<pl->man.cnt_entries; i++) {
		void* entry = get_next_entry(pl, &dft);
		switch (dft) {
		case DF_I32:
			pi32 = entry;
			//erwartete DatenTypen verarbeiten
			if (pi32->datatype == dt_bitmask) {
				if ((pi32->index == 0) && (pi32->value == 1))
					device_set_control(1);	// On-Bitmask
				else
					device_set_control(0);	//Off-Bitmask
			}
			if (pi32->datatype == dt_timer_sek) {
				device_set_timer(pi32->value);
			}
			//printf("I32: %d |IX: %d |DT: %d\n", pi32->value, pi32->index, pi32->datatype);
			break;

		case DF_I64:
			//pi64 = entry;
			//printf("I64: %lld |IX: %d\n", pi64->value, pi64->index);
			break;

		case DF_STR:
			//pstr = entry;
			//printf("STR: %s |IX: %d\n", pstr->txt, pstr->index);
			break;

		case DF_NULL:
			break;
		}
	}	//for entries


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

	//Ergebnis-Response-Queue
	measure_response_queue = xQueueCreate(MEASURE_QUEUE_SIZE, sizeof(uint32_t));

	nvs_flash_init();

    set_species(ACTOR);	//Repeater-Funktion kann in Ack-Frame zugewiesen werden (DIB im GW)

    device_init(measure_response_queue);

    wiog_wifi_actor_init();
	printf("Actor-UID: %d\n", my_uid);

	// Main-Loop - Statusmeldung
	while (true) {
		//falls zuvor auf 2 Dataframes das ACK ausgebleiben ist -> Channel-Scan veranlassen
		if (cnt_no_response_serie > 2) wifi_channel = 0;
		//ggf Channel-Scan veranlassen
		if (wifi_channel == 0) wiog_set_channel(0);

		uint32_t flag;
		/*BaseType_t res = */
		xQueueReceive(measure_response_queue, &flag, (interval_ms-1000) * MS);
		payload_t pl;
		bzero(&pl, sizeof(pl));
		set_management_data(&pl.man);

		uint32_t bm = device_get_out_bitmask();
		add_entry_I32(&pl, dt_bitmask, 0 ,0, bm);
		bm = device_get_in_bitmask();
		add_entry_I32(&pl, dt_bitmask, 1 ,0, bm);
		//Send Data to GW
		send_data_frame(&pl, pl.ix + sizeof(pl.man), ACTOR);
		//ACK mit aktuellem Interval abwarten
		vTaskDelay(1000*MS);
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


