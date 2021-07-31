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


#define VERSION  3
#define REVISION 0

//CallBack des Wiog-Headers nach Rx ACK, auch nach SNR-Data-Frame
void rx_ack_handler(wiog_header_t* pHdr) {
//	LED_STATUS_OFF;
	#ifdef DEBUG_X
		printf("Rx-ACK\n");
	#endif
}

//
void rx_data_handler(wiog_header_t* pHdr, payload_t* pl, int len)  {
hexdump((uint8_t*)pl, len);
//ShowDateTime(pl->man.utime);

	df_i32_t* pi32;
	df_i64_t* pi64;
	df_str_t* pstr;
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
				printf("BitMask: %d\n", pi32->value);
			}

			printf("I32: %d |IX: %d |DT: %d\n", pi32->value, pi32->index, pi32->datatype);
			break;

		case DF_I64:
			pi64 = entry;
			printf("I64: %8x |IX: %d |DT: %d\n", (uint32_t)pi64->value, pi64->index, pi64->datatype);
			break;

		case DF_STR:
			pstr = entry;
			printf("STR: %s |IX: %d\n", pstr->txt, pstr->index);
			break;

		case DF_NULL:
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

	cb_rx_ack_handler = &rx_ack_handler;
	cb_rx_data_handler = &rx_data_handler;

	wiog_wifi_actor_init();
	printf("UID: %05d\n", my_uid);

	while (true) {
		interval_ms = 10000;
		//falls zuvor auf 2 Dataframes das ACK ausgebleiben ist -> Channel-Scan veranlassen
		if (cnt_no_response_serie > 2) wifi_channel = 0;
		//ggf Channel-Scan veranlassen
		if (wifi_channel == 0) wiog_set_channel(0);

		uint32_t flag;
		//BaseType_t res =
//		xQueueReceive(measure_response_queue, &flag, (interval_ms-1000) * MS);
		vTaskDelay(interval_ms * MS);
		payload_t pl;
		bzero(&pl, sizeof(pl));
		set_management_data(&pl.man);

		/*
		 * Daten einsammeln + add_entry_XX
		 */
		add_entry_I32(&pl, 0, 0, 0, 12345);
		add_entry_I64(&pl, 1, 1, 1, 98765);
		add_entry_str(&pl, 2, 2, "*********************** #################### Hello Gatewy XYZ ####################*****************************");

		//Send Data to GW
//		send_data_frame(&pl, pl.ix + sizeof(pl.man), ACTOR);
/*
		#define SZ 65
		int sz = get_blocksize(SZ);
		uint8_t data[SZ];
		uint8_t crypt[sz];

		for (int i=0; i<SZ;i++) data[i] = i;
		bzero(crypt, sz);

		wiog_encrypt_data(data, crypt, SZ, 12345678);
		hexdump (crypt, sz);
		bzero(data, SZ);
		wiog_decrypt_data(crypt, data, sz, 12345678);

		hexdump(data, SZ);
*/


		//ACK mit aktuellem Interval abwarten
		vTaskDelay(1000*MS);
	}

}
