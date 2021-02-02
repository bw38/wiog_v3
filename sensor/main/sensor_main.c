#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "string.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#include "../../wiog_include/wiog_system.h"
#include "../../wiog_include/wiog_data.h"
#include "wiog_sensor.h"




void app_main(void)
{
	wiog_sensor_init();


	//Test-Frame senden ---------------------------------------------
	char txt[] = {"Hello World - How are you ? Dast ist ein Test"};

	uint8_t sz = strlen(txt) & 0xFF;
	uint8_t data[sz+1];				//Byte 0 => Längenbyte
	memcpy(&data[1], txt, sz);		//Byte 1 => Datenbereich
	data[0] = sz;

	bool b = send_data_frame(data);

	//----------------------------------------------------------------

	vTaskDelay (100*MS);
	rtc_onTime += esp_timer_get_time() / 1000;

	uint32_t sleeptime_ms = 3*1000;
	esp_sleep_enable_timer_wakeup(sleeptime_ms * 1000);
    rtc_gpio_isolate(GPIO_NUM_15); //Ruhestrom bei externem Pulldown reduzieren
    esp_deep_sleep_disable_rom_logging();
    printf("Goto DeepSleep for %dms\n", sleeptime_ms);
	esp_deep_sleep_start();

}

