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
#include "../../wiog_include/wiog_wifi_sensor.h"

#include "../../gadget/ds18b20/ds18b20.h"
#include "../../gadget/am2302/am2302.h"
#include "../../gadget/ubat/ubat.h"
#include "../../gadget/bme280/bme280_i2c_if.h"

#include "ulp_main.h"  //ulp_xxx.h wird automatisch generiert

#include "interface.h"

//Status-LED
#define LED_STATUS_INIT 	PIN_FUNC_SELECT(LED_STATUS_REG, PIN_FUNC_GPIO);	gpio_set_direction(LED_STATUS, GPIO_MODE_OUTPUT)
#define LED_STATUS_ON		gpio_set_level(LED_STATUS, 1)
#define LED_STATUS_OFF		gpio_set_level(LED_STATUS, 0)

#define VERSION  3
#define REVISION 0

//Measure-Response-Flags 2^n
#define RFLAG_UBAT		1
#define RFLAG_DS18B20	2
#define RFLAG_AM2302	4
#define RFLAG_BME280	8


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
	LED_STATUS_OFF;

	#ifdef DEBUG_X
		printf("[%04d]Rx-ACK\n", now());
	#endif
}

// --------------------------------------------------------------------------------------------------------------

// Wakeup-Stub - Code wird nach DeepSleep vor dem Boot-Prozess ausgeführt
void RTC_IRAM_ATTR esp_wake_deep_sleep(void) {
	//StepUp-Regler einschalten oder oben halten
 	GPIO_REG_WRITE(RTC_GPIO_OUT_W1TS_REG, 1<<(RTC_GPIO_OUT_DATA_W1TS_S + RTC_IO_STEPUP_CTRL));

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
    	//nach allgem. Reset initialisieren

    	//Systemvariablen aus NVS -> RTC-MEM
//    	for (int ix=0; ix < MAX_SYSVAR; ix++) rtc_sysvar[ix] = nvs_get_sysvar(ix);

    	init_ulp();	//ULP-Programm laden

    	//RTC-GPIO initialisieren ------------------------------------------
       	//Default - Steuerport StepUp-Regler, ctrl-port pulldwn
       	rtc_gpio_init(pwr_ctrl);
       	rtc_gpio_set_direction(pwr_ctrl, RTC_GPIO_MODE_OUTPUT_ONLY);
       	rtc_gpio_pullup_dis(pwr_ctrl);
       	rtc_gpio_pulldown_en(pwr_ctrl);

       	//individuelle Sensoren initialisieren
    	ds18b20_init(GPIO_DS18B20_OWP);
//    	am2302_init(AM2302_GPIO_OWP);
    	// ----------------------------------------------------------------
    } else {
    	//aus DeepSleep
    	waked_up = true;
    }

	LED_STATUS_INIT;
	LED_STATUS_ON;

	//Ergebnis-Response-Queue
	measure_response_queue = xQueueCreate(MEASURE_QUEUE_SIZE, sizeof(uint32_t));

	//Batteriemessung, Mess- und Steuerport initialisieren
	//ADC1-Chn / GPIO-GND Spannungsteiler, ohne = -1 / Anzahl Samples (100 => ca. 4.3ms)
	ubat_init(UBAT_ADC_CHN, UBAT_DIV_GND, 100);

    //bei Erstinbetriebnahme eines Sensors (ESP32 Rev 01) kann hier die Vref in Sensor geschrieben werden
	//ggf in sdkconfig TP-Values und VRef ausschalten, wenn vorcalibrierter Wert nicht passt
	//vref kann auch über 1200 liegen
	ubat_set_vref(1180);

	//BME280 - I2C-Interface
	esp_err_t err = bme280_i2c_master_init(BME280_I2C_MASTER_NUM, BME280_I2C_MASTER_SDA_IO, BME280_I2C_MASTER_SCL_IO, waked_up);
	if (err == ESP_OK)
		bme280_i2c_start(RFLAG_BME280);

	//Wifi-Initialisierung -----------------------------------------------
	#ifdef DEBUG_X
		printf("[%04d]Init Wifi\n", now());
	#endif
	wiog_wifi_sensor_init();	//ca. 50ms !!!
	#ifdef DEBUG_X
		printf("[%04d]Wifi ready\n", now());
		printf("[%04d]UID: %05d\n", now(), my_uid);
	#endif
	// ------------------------------------------------------------------

	//Batteriespannungsmessung mit Wifi-Last starten
	ubat_start(RFLAG_UBAT);
	//vTaskDelay(1500*MS);			//Test ADC-Kalibrierung

	//Temp-Messung in ULP bereits erledigt, hier nur RFlag in die Response-Queue stellen
	ds18b20_start(RFLAG_DS18B20);

	//Temp-Messung in ULP bereits erledigt, hier nur RFlag in die Response-Queue stellen
	//am2302_start(RFLAG_AM2302);


	//Einsammeln der Messergebnisse ------------------------------------------
	payload_t pl;
	bzero(&pl, sizeof(pl));
	set_management_data(&pl.man);

	//Kernlaufzeit
	add_entry_I32(&pl, dt_runtime_ms, 0, 0, rtc_onTime);
	//Cycle
	add_entry_I32(&pl, dt_cycle, 0, 0, ++rtc_cycles);

	//Maske aller Gadget-Flags
	uint32_t flags = RFLAG_UBAT |
					 RFLAG_DS18B20 |
					 RFLAG_BME280;

	uint32_t flag = 0;	//in der Queue zurückgeliefertes Einzelflag
	//Gadgets melden mit Response-Flage die Bereitstellung der Messergebnisse
	while ((xQueueReceive(measure_response_queue, &flag, 200*MS) == pdTRUE)) {
		if ((flag & RFLAG_UBAT) != 0){	//Ergebnis Batteriespannungsmessung
			uint32_t res = ubat_get_result();
			uint32_t ures = (int)((res + (res * UBAT_DIVIDER)));
			add_entry_I32(&pl, dt_ubat_mv, 0, 0, ures);
			#ifdef DEBUG_X
				printf("[%04d]ADC: %dmV | UBat: %dmV\n", now(), res, ures);
			#endif
		}	// UBat

		else
		if ((flag & RFLAG_DS18B20) != 0){	//ds18b20-Ergebnis
			ds18b20_result_t res = ds18b20_get_result();
			add_entry_I32(&pl, dt_ds18b20, 0, res.status, res.temperature);
			#ifdef DEBUG_X
				printf("[%04d]DS18B20 Temp: %d\n", now(), res.temperature);
			#endif
		}	// DB18B20
/*		else
		if ((flag & RFLAG_AM2302) != 0){	// DHT21
			am2302_result_t res = am2302_get_result();
			add_entry_I32(&pl, dt_am2302, 0, res.crc_check, res.temperature);
			add_entry_I32(&pl, dt_am2302, 1, res.crc_check, res.humidity);
			#ifdef DEBUG_X
				printf("[%04d]DS18B20 Temp: %d | Humi: %d\n", now(), res.temperature, res.humidity);
			#endif
		}	// DHT21
*/

		else
		if ((flag & RFLAG_BME280) != 0) {
			if (rtc_bme280_init_result == 0) {
				bme280_result_t res = bme280_i2c_get_result();
				add_entry_I32(&pl, dt_bme280, 0, res.status, res.pressure);
				add_entry_I32(&pl, dt_bme280, 1, res.status, res.temperature);
				add_entry_I32(&pl, dt_bme280, 2, res.status, res.humidity);

				#ifdef DEBUG_X
					printf("[%04d]Press: %d | Temp: %d | Humi: %d | Status: %d\n", now(),
							res.pressure, res.temperature, res.humidity, res.status);
				#endif
			}
			else {
				add_entry_I32(&pl, dt_bme280, 0, 1, 0);	//Fehlermeldung Status = 1, Pressure = 0
				printf ("Fehler BME280\n");
			}
		} //bme280

		// .....

		flags &= ~flag;
		if (flags == 0) break;
	}

	if (flags != 0)  { //Funktionsfehler

	}

	//Send Data to GW
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

