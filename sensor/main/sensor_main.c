#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "string.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#include "esp32/ulp.h"
#include "soc/rtc.h"

#include "../../wiog_include/wiog_system.h"
#include "../../wiog_include/wiog_data.h"
#include "../../wiog_include/wiog_wifi.h"

#include "../../devices/ds18b20/ds18b20.h"

#include "ulp_main.h"  //ulp_xxx.h wird automatisch generiert

#include "interface.h"



#define VERSION  3
#define REVISION 0

//Definitions ULP-Programm
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

const gpio_num_t pwr_ctrl = GPIO_STEPUP_CTRL;	//Steuerung StepUp-Regler

//SystemVariablen zur Steurung der Sensoren
//#define MAX_SYSVAR  8 //Systemvariablen [0..7], Data-Entries im Quittungspaket
//RTC_DATA_ATTR static int32_t   rtc_sysvar[MAX_SYSVAR] __attribute__((unused)); 	//0..7

//Prototypen
static void init_ulp();

//---------------------------------------------------------------------------------------------------------------

//CallBack des Wiog-Headers nach Rx ACK, vor DeepSleep
void rx_data_handler(wiog_header_t* pHdr)  {
	printf("Rx - ACK\n");
//	hexdump((uint8_t*) pHdr, sizeof(wiog_header_t));
}


void init_rtc(void)  {
   	//Default - Steuerport StepUp-Regler, ctrl-port pulldwn
   	rtc_gpio_init(pwr_ctrl);
   	rtc_gpio_set_direction(pwr_ctrl, RTC_GPIO_MODE_OUTPUT_ONLY);
   	rtc_gpio_pullup_dis(pwr_ctrl);
   	rtc_gpio_pulldown_en(pwr_ctrl);

   	//individuelle Sensoren initialisieren
	ds18b20_init(GPIO_DS18B20_OWP);
}


// --------------------------------------------------------------------------------------------------------------

// Wakeup-Stub - Code wird nach DeepSleep vor dem Boot-Prozess ausgeführt
void RTC_IRAM_ATTR esp_wake_deep_sleep(void) {
	//StepUp-Regler einschalten oder oben halten
// 	GPIO_REG_WRITE(RTC_GPIO_OUT_W1TS_REG, 1<<(RTC_GPIO_OUT_DATA_W1TS_S + RTC_IO_STEPUP_CTRL));

 	//Extra Delay in Wakeup Stub -> s. SDK-Config/ESP32-specific 0..5000µs
 	esp_default_wake_deep_sleep();
}

void app_main(void) {

	version = VERSION;
	revision = REVISION;

	cb_rx_handler = &rx_data_handler;

	init_nvs();

	bool waked_up = false;
	esp_sleep_wakeup_cause_t rst_reason = esp_sleep_get_wakeup_cause();

	#ifdef DEBUG_X
		printf("[%04d]RST-Cause: %d\n", now(), rst_reason);
	#endif

	if ((rst_reason != ESP_SLEEP_WAKEUP_ULP)  &&
    	(rst_reason != ESP_SLEEP_WAKEUP_EXT1) &&
		(rst_reason != ESP_SLEEP_WAKEUP_TIMER)) {
    	//frisch initialisieren
    	//Systemvariablen aus NVS -> RTC-MEM
//    	for (int ix=0; ix < MAX_SYSVAR; ix++) rtc_sysvar[ix] = nvs_get_sysvar(ix);

    	init_rtc();	//RTC-GPIO initialisieren
    	init_ulp();	//ULP-Programm laden

    } else {
    	waked_up = true;
    }


	measure_response_queue = xQueueCreate(MEASURE_QUEUE_SIZE, sizeof(uint32_t));

	ds18b20_start(1);

	#ifdef DEBUG_X
		printf("[%04d]Init Wifi\n", now());
	#endif
	wiog_wifi_sensor_init();	//ca. 50ms !!!
	#ifdef DEBUG_X
		printf("[%04d]Wifi ready\n", now());
		printf("[%04d]UID: %05d\n", now(), my_uid);
	#endif

	payload_t pl;
	bzero(&pl, sizeof(pl));

	add_entry_I32(&pl, dt_runtime_ms, 0, 0, rtc_onTime);

	uint32_t flags = 1;	//Maske aller Flags
	uint32_t flag = 0;	//in der Queue zurückgeliefertes Einzelflag

	while ((xQueueReceive(measure_response_queue, &flag, 200*MS) == pdTRUE)) {

		if ((flags & 1) != 0){	//ds18b20-Ergebnis
			add_entry_I32(&pl, dt_ds18b20, 0, 0, ds18b20_temperature);
			flags &= ~flag;
			printf("[%04d]Temp: %d\n", now(), ds18b20_temperature);
		}

		if (flags == 0) break;
	}

	if (flags != 0)  { //Funktionsfehler

	}

	//Data to GW
	send_data_frame(&pl, pl.ix + sizeof(pl.man));

	wiog_wifi_sensor_goto_sleep(WS_ULP);

	//----------------------------------------------------------------

}


void init_ulp() {

#ifdef DEBUG_X
	//ULP-Clockspeed
	uint32_t rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 100);
	if (rtc_8md256_period > 0) {	//nach esp_restart() => 0
		uint32_t rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period;
		printf("RTC FastFreq: %dHz\n", rtc_fast_freq_hz);
	}
   	printf("[%04d]Initializing ULP\n", now());
#endif
   	//ULP-Programm laden
   	esp_err_t err = ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
   	ESP_ERROR_CHECK(err);

   	printf("ULP-Program-Size: %d\n", ulp_main_bin_end - ulp_main_bin_start);

   	//nicht während 1. Hauptzyklus aufwachen
   	//ULP wird trotzdem 1x ausgeführt ??
   	ESP_ERROR_CHECK(ulp_set_wakeup_period(0, 3000 *1000));
   	//ULP-Programm starten
   	ESP_ERROR_CHECK(ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t)));
}

