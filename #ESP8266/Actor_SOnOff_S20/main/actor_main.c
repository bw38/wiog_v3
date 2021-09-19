/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "strings.h"

#include "wiog_system.h"
#include "wiog_data.h"
#include "wiog_wifi_actor.h"

#include "sonoff.h"


#define VERSION  3
#define REVISION 0

//CallBack des Wiog-Headers nach Rx ACK, auch nach SNR-Data-Frame
void rx_ack_handler(wiog_header_t* pHdr) {
	device_set_status_led(0);
	#ifdef DEBUG_X
		printf("Rx-ACK\n");
	#endif
}


void rx_data_handler(wiog_header_t* pHdr, payload_t* pl, int len)  {
hexdump((uint8_t*)pl, len);
//ShowDateTime(pl->man.utime);

	df_i32_t* pi32;
	pl->ix = 0;
	data_frame_t dft;

	for (int i=0; i<pl->man.cnt_entries; i++) {
printf("I: %d\n", i);
		void* entry = get_next_entry(pl, &dft);
		if (entry == NULL) break;

		switch (dft) {
		case DF_I32:
			pi32 = entry;
			//erwartete DatenTypen verarbeiten
			if (pi32->datatype == dt_bitmask) {
				device_set_ns_state(pi32->value);
			}
			if (pi32->datatype == dt_timer_sek) {
				device_set_ns_timer(pi32->value);
			}
			#ifdef DEBUG_X
				printf("Rx - DT: %d | Ix: %d | Status: %d | Val: %d\n",
						pi32->datatype, pi32->index, pi32->status, pi32->value);
			#endif
			break;
		default:
			printf("Wrong Datatype\n");
			break;
		}
	}	//for entries


	#ifdef DEBUG_X
		printf("Rx-Data\n");
	#endif
}


void app_main()
{
    // Print chip information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP8266 chip with %d CPU cores, WiFi, ", chip_info.cores);

	version = VERSION;
	revision = REVISION;

	measure_response_queue = xQueueCreate(10, sizeof(uint32_t));

	cb_rx_ack_handler = &rx_ack_handler;
	cb_rx_data_handler = &rx_data_handler;

	wiog_wifi_actor_init();
	printf("UID: %05d\n", my_uid);

	device_init();

	while (true) {
		//falls zuvor auf 2 Dataframes das ACK ausgebleiben ist -> Channel-Scan veranlassen
		if (cnt_no_response_serie > 2) wifi_channel = 0;
		//ggf Channel-Scan veranlassen
		if (wifi_channel == 0) wiog_set_channel(0);

		//Inverval- oder Ereignissteurung
		uint32_t flag;
		xQueueReceive(measure_response_queue, &flag, (interval_ms-1000) * MS);

		payload_t pl;
		bzero(&pl, sizeof(pl));
		set_management_data(&pl.man);

		uint32_t bm1 = device_get_ns_state();
		add_entry_I32(&pl, dt_bitmask, 0 ,0, bm1);
		uint32_t bm2 = device_get_button_state();
		add_entry_I32(&pl, dt_bitmask, 1 ,0, bm2);
		#ifdef DEBUG_X
			printf("NS: %d | BT: %d\n", bm1, bm2);
		#endif
		//Send Data to GW
		device_set_status_led(1);
		send_data_frame(&pl, pl.ix + sizeof(pl.man), ACTOR);

		//ACK mit aktuellem Interval abwarten
		vTaskDelay(1000*MS);
	}

}
