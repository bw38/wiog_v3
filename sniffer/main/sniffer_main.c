/**

 * ESP32/016
 * WiFi Sniffer.
 */

#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "string.h"
#include "esp_log.h"

#include "../../wiog_include/wiog_system.h"

#define WORKING_CHANNEL 3

#define	LED_GPIO_PIN		GPIO_NUM_4
#define	WIFI_CHANNEL_MAX	13

static wifi_country_t wifi_country = {.cc="DE", .schan=1, .nchan=13, .policy=WIFI_COUNTRY_POLICY_AUTO};

static uint32_t loop_cnt = 0;

static int64_t ts_pkts_start = 0;

#define WIOG_SNIFFER_QUEUE_SIZE 6
static xQueueHandle wiog_sniffer_queue;

uint32_t last_frame_id;


//Wifi-Rx-Callback im Sniffermode - Daten in die Rx-Queue stellen
IRAM_ATTR  void wifi_sniffer_packet_cb(void* buff, wifi_promiscuous_pkt_type_t type) {
	const wifi_promiscuous_pkt_t   *ppkt = (wifi_promiscuous_pkt_t *)buff;
	const wiog_data_frame_t  *ipkt = (wiog_data_frame_t *)ppkt->payload;
	const wiog_header_t *header =  &ipkt->header;

	//nur Pakete des eigenen Netzes bearbeiten
	if (memcmp(header->mac_net, &mac_net, sizeof(mac_addr_t)) !=0) {
		return;
	}

	wiog_event_rxdata_t frame;
	frame.timestamp = now()*1000;
	memcpy(&frame.rx_ctrl, &ppkt->rx_ctrl, sizeof(wifi_pkt_rx_ctrl_t));
	memcpy(&frame.wiog_hdr, header, sizeof(wiog_header_t));
	frame.data_len = ppkt->rx_ctrl.sig_len - sizeof(wiog_header_t) - 4; //ohne FCS
	frame.data = (uint8_t*)malloc(frame.data_len);
	memcpy(frame.data, &ipkt->data, frame.data_len);
	if (xQueueSend(wiog_sniffer_queue, &frame, portMAX_DELAY) != pdTRUE) {
			ESP_LOGW("Sniffer: ", "receive queue fail");
			free(frame.data);
	}
}


static void wiog_sniffer_task(void *pvParameter) {

	wiog_event_rxdata_t evt;
	bool bcr;

	while ((xQueueReceive(wiog_sniffer_queue, &evt, portMAX_DELAY) == pdTRUE)) {

		wifi_pkt_rx_ctrl_t *pRx_ctrl = &evt.rx_ctrl;
		wiog_header_t *pHdr = &evt.wiog_hdr;

		bcr = (pHdr->frameid != last_frame_id);
		if (bcr) {
			printf("===> %.0fms\n", (evt.timestamp-ts_pkts_start) / 1000.0);
			ts_pkts_start = evt.timestamp;
		}

		printf("%02x => %02x | ", pHdr->mac_from[5], pHdr->mac_to[5]);

		printf("snr:%02ddB ch:%02d | L%.03d | ",
			pRx_ctrl->rssi - pRx_ctrl->noise_floor,
			pHdr->channel,
			pRx_ctrl->sig_len
		);

		printf(" uid:%05d | typ:%02x | A:%02x | B:%02x | C:%05d | fid:%08x  | sc%04x |",
			pHdr->uid,
			pHdr->vtype,
			pHdr->tagA,
			pHdr->tagB,
			(uint16_t)pHdr->tagC,
			pHdr->frameid,
			pHdr->seq_ctrl);


		last_frame_id = pHdr->frameid;

		free(evt.data);

		printf("| %.0fms\n", (evt.timestamp-ts_pkts_start) / 1000.0);



		loop_cnt = 0;

	}
}


void app_main(void) {

	uint8_t channel = WORKING_CHANNEL;

	nvs_flash_init();


	//Rx-Queue -> Verarbeitung empfangener Daten
	wiog_sniffer_queue = xQueueCreate(WIOG_SNIFFER_QUEUE_SIZE, sizeof(wiog_event_rxdata_t));
	xTaskCreate(wiog_sniffer_task, "wiog_rx_task", 4096, NULL, 12, NULL);

	esp_netif_init();
   	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
   	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_country(&wifi_country) ); // set country for channel range [1 .. 13]
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR));
	esp_wifi_set_promiscuous(true);
	esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_cb);
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
	ESP_ERROR_CHECK( esp_wifi_start() );
	esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

	wifi_promiscuous_filter_t filter;
	filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
	ESP_ERROR_CHECK( esp_wifi_set_promiscuous_filter(&filter) );

	while (true) {
		vTaskDelay(100*MS);
		if (loop_cnt++ == 5) {
//			printf("\n");
		}
	}
}

// -------------------------------------------------------------------------

