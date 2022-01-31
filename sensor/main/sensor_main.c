#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "string.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp32/ulp.h"
#include "soc/rtc.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp32s2/ulp_riscv.h"
#include <math.h>

#include "wiog_include/wiog_system.h"
#include "wiog_include/wiog_data.h"
#include "wiog_include/wiog_wifi_sensor.h"

#include "ulp_main.h"  //ulp_xxx.h wird automatisch generiert

#include "interface.h"

//Gerätesprzifische Header f. Mainprocess
#ifdef RFLAG_UBAT
#include "gadget/ubat/ubat.h"
#endif
#ifdef RFLAG_BME280
#include "gadget/bme280/bme280_i2c_if.h"
#endif
#ifdef RFLAG_SHT31
#include "gadget/sht31/sht31.h"
#endif
#ifdef RFLAG_SHTC3
#include "gadget/shtc3/shtc3_i2c_if.h"
#endif


#ifdef USE_STATUS_LED
//Status-LED
#define LED_STATUS_INIT 	PIN_FUNC_SELECT(LED_STATUS_REG, PIN_FUNC_GPIO);	gpio_set_direction(LED_STATUS, GPIO_MODE_OUTPUT)
#define LED_STATUS_ON		gpio_set_level(LED_STATUS, 1)
#define LED_STATUS_OFF		gpio_set_level(LED_STATUS, 0)
#endif

#define VERSION  3
#define REVISION 0

RTC_DATA_ATTR uint32_t rtc_fsm_cycles;

#define MEASURE_QUEUE_SIZE 32
xQueueHandle measure_response_queue;	//Flags -> Mess-Ereignis an main

i2c_port_t i2c_mport = -1;

//SystemVariablen zur Steurung der Sensoren
//#define MAX_SYSVAR  8 //Systemvariablen [0..7], Data-Entries im Quittungspaket
//RTC_DATA_ATTR static int32_t   rtc_sysvar[MAX_SYSVAR] __attribute__((unused)); 	//0..7


//---------------------------------------------------------------------------------------------------------------

//CallBack des Wiog-Headers nach Rx ACK, vor DeepSleep
void rx_data_handler(wiog_header_t* pHdr)  {
	#ifdef USE_STATUS_LED
	LED_STATUS_OFF;
	#endif

	#ifdef RFLAG_BME280_RISCV
	ulp_set_bme280_thres_temp = pHdr->rdi8A;		//A * 0.01°C	[0..2.55°C]
	ulp_set_bme280_thres_humi = pHdr->rdi8B * 100;	//B * 0.1%		[0..25.5%]
	ulp_set_bme280_thres_pres = pHdr->rdi8C * 10;	//C * 0.1hPa	[0..25.5hPa]
	ulp_set_bme280_force_wake = pHdr->rdi8D;
		#ifdef DEBUG_X
		printf("Thresholds: %.2f°C | %.2f%% | %.2fhPa\n",
				ulp_set_bme280_thres_temp / 100.0, ulp_set_bme280_thres_humi / 1000.0, ulp_set_bme280_thres_pres / 100.0);
		printf("Max ULP Cycles: %d\n", ulp_set_bme280_force_wake);
		#endif
	#endif

	#ifdef RFLAG_SHT31_RISCV
	ulp_set_sht31_thres_temp = pHdr->rdi8A;			//A * 0.01°C	[0..2.55°C]
	ulp_set_sht31_thres_humi = pHdr->rdi8B * 10;	//B * 0.1%		[0..25.5%]
	ulp_sht31_set_heater = pHdr->rdi8C;				//0 - Heizung aus / !=0 - Heizung f. einen Messzyklus an
	ulp_set_sht31_force_wake = pHdr->rdi8D;
		#ifdef DEBUG_X
		printf("Thresholds: %.2f°C | %.2f%%\n",
				ulp_set_sht31_thres_temp / 100.0, ulp_set_sht31_thres_humi / 100.0);
		printf ("Heater: %d\n", ulp_sht31_set_heater);
		printf("Max ULP Cycles: %d\n", ulp_set_sht31_force_wake);
		#endif
	#endif

	#ifdef RFLAG_AM2302_FSM
	ulp_temp_threshold = pHdr->rdi8A;			//A * 0.1°C	[0..25.5°C]
	ulp_humi_threshold = pHdr->rdi8B;			//B * 0.1%	[0..25.5%]
	ulp_ncycles_force_wake = pHdr->rdi8D;
		#ifdef DEBUG_X
		printf("Thresholds: %.1f°C | %.1f%%\n",
				ulp_temp_threshold / 10.0, ulp_humi_threshold / 10.0);
		printf("Max ULP Cycles: %d\n", ulp_ncycles_force_wake);
		#endif
	#endif

	#ifdef RFLAG_DS18B20_FSM
		ulp_temp_threshold = round ((pHdr->rdi8A / 100.0) * 16.0);			//A * 0.01°C	[0..2.55°C]
		ulp_ncycles_force_wake = pHdr->rdi8D;		//nach n ulp-cycles zwangsaufwecken
		#ifdef DEBUG_X
		printf("Thresholds: %.1f°C\n", ulp_temp_threshold / 16.0);
		printf("Max ULP Cycles: %d\n", ulp_ncycles_force_wake);
		#endif
	#endif

	#ifdef DEBUG_X
		printf("[%04d]Rx-ACK\n", now());

	#endif
}

// --------------------------------------------------------------------------------------------------------------


#ifdef USE_I2C_MASTER
i2c_port_t i2c_master_init()
{
	//I2C-Schnittstelle initialisieren
	i2c_port_t mport = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.clk_flags = 0; // ab IDF > v4.1
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;	//10k pullup auf Sensorboard
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;	//10k Pullup auf Sensorboard
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(mport, &conf);
    if (i2c_driver_install(mport, conf.mode, 0, 0, 0) == 0) {
#ifdef DEBUG_X
    	printf("[%04d] I2C-Config Ok\n", now());
#endif
    } else {
    	i2c_mport = -1;
    	ESP_LOGE("I2C", "Init I2C failed");
    }
	return mport;
}
#endif

#ifdef USE_STEPUP_CTRL
const gpio_num_t pwr_ctrl = GPIO_STEPUP_CTRL;	//Steuerung StepUp-Regler
void init_stepup_ctrl(void)
{
	//RTC-GPIO initialisieren ------------------------------------------
   	//Default - Steuerport StepUp-Regler, ctrl-port pulldwn
   	rtc_gpio_init(pwr_ctrl);
   	rtc_gpio_set_direction(pwr_ctrl, RTC_GPIO_MODE_OUTPUT_ONLY);
   	rtc_gpio_pullup_dis(pwr_ctrl);
   	rtc_gpio_pulldown_en(pwr_ctrl);
   	//Stepup-Regler hochtasten nach allgem. Reset
   	//nach DeepSleep bereits oben durch ULP / Wakeup-Stub
   	rtc_gpio_set_level(pwr_ctrl, 1);
}
#endif


#ifdef USE_ULP_FSM
//Definitions ULP-Programm
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

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
   	//ULP wird trotzdem 1x ausgeführt
   	ESP_ERROR_CHECK(ulp_set_wakeup_period(0, 3000 *1000));
   	//ULP-Programm starten
   	ESP_ERROR_CHECK(ulp_run(&ulp_entry - RTC_SLOW_MEM));
}
#endif

#ifdef USE_ULP_RISCV
//Definitions ULP-Programm
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

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
    esp_err_t err = ulp_riscv_load_binary(ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start));
    ESP_ERROR_CHECK(err);

   	printf("ULP-Program-Size: %d\n", ulp_main_bin_end - ulp_main_bin_start);

   	//nicht während 1. Hauptzyklus aufwachen
   	//ULP wird trotzdem 1x ausgeführt
   	ESP_ERROR_CHECK(ulp_set_wakeup_period(0, 3000 *1000));
   	//ULP-Programm starten
   	ESP_ERROR_CHECK(ulp_riscv_run());
}
#endif

// --------------------------------------------------------------------------------------------------------------

// Wakeup-Stub - Code wird nach DeepSleep vor dem Boot-Prozess ausgeführt
void RTC_IRAM_ATTR esp_wake_deep_sleep(void) {
	//StepUp-Regler einschalten oder oben halten
	#ifdef USE_STEPUP_CTRL
 	GPIO_REG_WRITE(RTC_GPIO_OUT_W1TS_REG, 1<<(RTC_GPIO_OUT_DATA_W1TS_S + RTC_IO_STEPUP_CTRL));
	#endif

 	//Extra Delay in Wakeup Stub -> s. SDK-Config/ESP32-specific 0..5000µs
 	esp_default_wake_deep_sleep();
}

void app_main(void) {

	tStart = esp_timer_get_time();
	cb_rx_handler = &rx_data_handler;

	init_nvs();

	bool waked_up __attribute__((unused)) = false;
	esp_sleep_wakeup_cause_t rst_reason = esp_sleep_get_wakeup_cause();

	#ifdef DEBUG_X
	printf("\n");
	printf("[%04d]Init NVS\n", now());
	printf("[%04d]RST-Cause: %d\n", now(), rst_reason);
	#endif

	if ((rst_reason != ESP_SLEEP_WAKEUP_ULP)  &&
    	(rst_reason != ESP_SLEEP_WAKEUP_EXT1) &&
		(rst_reason != ESP_SLEEP_WAKEUP_TIMER)) {
    	//nach allgem. Reset initialisieren

    	//Systemvariablen aus NVS -> RTC-MEM
//    	for (int ix=0; ix < MAX_SYSVAR; ix++) rtc_sysvar[ix] = nvs_get_sysvar(ix);

		#if defined (USE_ULP_FSM) || defined (USE_ULP_RISCV)
    	init_ulp();	//ULP-Programm laden
		#endif

		#ifdef USE_STEPUP_CTRL
    	init_stepup_ctrl();
		#endif

    	// ----------------------------------------------------------------
    } else {
    	//aus DeepSleep
    	waked_up = true;
    }

	#ifdef USE_STATUS_LED
	LED_STATUS_INIT;
	LED_STATUS_ON;
	#endif

	//Messungen initialisieren
	//Ergebnis-Response-Queue
	measure_response_queue = xQueueCreate(MEASURE_QUEUE_SIZE, sizeof(uint32_t));
	uint32_t flags = 0;	//Maske aller Gadget-Flags


	#ifdef RFLAG_UBAT
	//Batteriemessung, Mess- und Steuerport initialisieren
	flags |=  RFLAG_UBAT;
	ubat_init(UBAT_ADC_CHN, UBAT_DIV_GND, 50, VREF);
	//später starten
	#endif

	#ifdef RFLAG_DS18B20_FSM
	if (waked_up) {	//nur nach Kaltstart initialisieren
		rtc_gpio_init(GPIO_DS18B20_OWP);
		rtc_gpio_set_direction(GPIO_DS18B20_OWP, RTC_GPIO_MODE_INPUT_ONLY);
		rtc_gpio_pullup_en(GPIO_DS18B20_OWP);		//Dataport intern pullup
		rtc_gpio_pulldown_dis(GPIO_DS18B20_OWP);
		rtc_gpio_hold_dis(GPIO_DS18B20_OWP);
	}
	//bedingtes wakeup
	ulp_temp_threshold = TEMP_THRESHOLD * 16;
	ulp_ncycles_force_wake = MAX_FORCE_REPORT;
	//Temp-Messung in ULP bereits erledigt,  nur RFlag in die Response-Queue stellen
	flags |=  RFLAG_DS18B20_FSM;
	uint32_t f_ds18b20 = RFLAG_DS18B20_FSM;
	xQueueSend(measure_response_queue, &f_ds18b20, portMAX_DELAY);
	#endif

	#ifdef RFLAG_DS18B20_RISCV
	if (waked_up) ulp_owp = GPIO_DS18B20_OWP;
	ulp_temp_threshold = TEMP_THRESHOLD * 16;
	ulp_max_force_report = MAX_FORCE_REPORT;
	//Temp-Messung in ULP bereits erledigt,  nur RFlag in die Response-Queue stellen
	flags |=  RFLAG_DS18B20_RISCV;
	ds18b20_start(measure_response_queue, RFLAG_DS18B20_RISCV);
	#endif

	#ifdef RFLAG_AM2302_FSM
	//Temp-Messung in ULP bereits erledigt, hier nur RFlag in die Response-Queue stellen
	if (!waked_up) {
	   	rtc_gpio_init(AM2302_GPIO_OWP);
	  	rtc_gpio_set_direction(AM2302_GPIO_OWP, RTC_GPIO_MODE_INPUT_ONLY);
	  	rtc_fsm_cycles = 0;
	  	ulp_cycles = 0;
	}
	ulp_temp_threshold = 0;	//wird mit ReturnValue im ACK neu gesetzt
	ulp_humi_threshold = 0;
	ulp_ncycles_force_wake = 0;
	flags |= RFLAG_AM2302_FSM;
	uint32_t f_am2302 = RFLAG_AM2302_FSM;
	xQueueSend(measure_response_queue, &f_am2302, portMAX_DELAY);
	#endif

	#ifdef RFLAG_BME280
	//I2C-Interface initialisieren
	if (i2c_mport < 0) i2c_mport = i2c_master_init();
	if (i2c_mport >= 0) {
		//Messung initialisieren und starten
		if (bme280_i2c_init(i2c_mport, waked_up) == 0) {
			bme280_i2c_start(measure_response_queue, RFLAG_BME280);
		} else {
			ESP_LOGE("BME280", "Init Error");
		}
		flags |= RFLAG_BME280; // in Queue auf Ergebnis warten
	}
	#endif	//BME280


	#ifdef RFLAG_SHTC3
	//I2C-Interface initialisieren
	if (i2c_mport < 0) i2c_mport = i2c_master_init();
	if (i2c_mport >= 0) {
		//SHT3 - Initialisieren und Messung starten
		if (shtc3_i2c_init(i2c_mport, waked_up) == ESP_OK) {
			shtc3_i2c_start(measure_response_queue, RFLAG_SHTC3);
			flags |= RFLAG_SHTC3;
		}	else {
			ESP_LOGE("SHTC3", "Init Error");
		}
	}
	#endif

	#ifdef RFLAG_SHT31
	//I2C-Interface initialisieren
	if (i2c_mport < 0) i2c_mport = i2c_master_init();
	if (i2c_mport >= 0) {
		//SHT31 - Initialisieren und Messung starten
		if (sht31_i2c_init(i2c_mport, waked_up) == ESP_OK) {
			sht31_i2c_start(measure_response_queue, RFLAG_SHT31);
			flags |= RFLAG_SHT31;
		}	else {
			ESP_LOGE("SHT31", "Init Error");
		}
	}
	#endif

	#ifdef RFLAG_SHT31_RISCV
	ulp_sht31_set_heater = 0;	//	1 => testweise einschalten
	ulp_sht31_set_mode = 1;	//  0-Low, 1-Medium, 2-High
	ulp_sht31_sda = I2C_MASTER_SDA_IO;
	ulp_sht31_scl = I2C_MASTER_SCL_IO;
	flags |=  RFLAG_SHT31_RISCV;
	uint32_t f_sht31_ulp = RFLAG_SHT31_RISCV;
	//Ergebiss bereuts vorhanden
	xQueueSend(measure_response_queue, &f_sht31_ulp, portMAX_DELAY);
	#endif

	#ifdef RFLAG_BME280_RISCV
	//Aufweckbedingungen f. nächsten Messzyklus
//	ulp_set_bme280_force_wake = MAX_FORCE_REPORT;
//	ulp_set_bme280_thres_temp = TEMP_THRESHOLD * 100;
//	ulp_set_bme280_thres_humi = HUMI_THRESHOLD * 1000;
//	ulp_set_bme280_thres_pres = PRES_THRESHOLD * 100;
	ulp_bme280_sda = I2C_MASTER_SDA_IO;
	ulp_bme280_scl = I2C_MASTER_SCL_IO;
	flags |=  RFLAG_BME280_RISCV;
	uint32_t f_bme280_ulp = RFLAG_BME280_RISCV;
	//Ergebnis bereits vorhanden
	xQueueSend(measure_response_queue, &f_bme280_ulp, portMAX_DELAY);
	#endif


	//Wifi-Initialisierung -----------------------------------------------
	#ifdef DEBUG_X
		printf("[%04d]Init Wifi\n", now());
	#endif
	wiog_wifi_sensor_init();	//ca. 25ms
	#ifdef DEBUG_X
		printf("[%04d]Wifi ready\n", now());
		printf("[%04d]UID: %05d\n", now(), my_uid);
	#endif
	// ------------------------------------------------------------------

	//Batteriespannungsmessung mit Wifi-Last starten
	#ifdef RFLAG_UBAT
	ubat_start(measure_response_queue, RFLAG_UBAT);
	//vTaskDelay(1500*MS);			//Test ADC-Kalibrierung
	#endif

	//Einsammeln der Messergebnisse ------------------------------------------
	payload_t pl;
	bzero(&pl, sizeof(pl));
	set_management_data(&pl.man);

	uint32_t flag = 0;	//in der Queue zurückgeliefertes Einzelflag
	//Gadgets melden mit Response-Flage die Bereitstellung der Messergebnisse
	while ((xQueueReceive(measure_response_queue, &flag, 200*MS) == pdTRUE)) {
		#ifdef RFLAG_UBAT
		if ((flag & RFLAG_UBAT) != 0){	//Ergebnis Batteriespannungsmessung
			uint32_t res = ubat_get_result();
			uint32_t ures = (int)((res + (res * UBAT_DIVIDER)));
			add_entry_I32(&pl, dt_ubat_mv, 0, 0, ures);
			#ifdef DEBUG_X
				printf("[%04d]ADC: %dmV | UBat: %dmV\n", now(), res, ures);
			#endif
		}	// UBat
		#endif

		#ifdef RFLAG_DS18B20_FSM
		if ((flag & RFLAG_DS18B20_FSM) != 0){	//ds18b20-Ergebnis
			int32_t temperature = (((int32_t)(ulp_temp_raw) & UINT16_MAX)*100) / 16.0;
			int8_t status = ulp_crc_err;
			add_entry_I32(&pl, dt_ds18b20, 0, status, temperature);
			rtc_fsm_cycles += ulp_cycles & 0xFFFF;	//Gesamtzähler ulp-cycles
			ulp_cycles = 0;	//Rücksetzen f. nächsten Maincyle
			pl.man.ulp_cycles = rtc_fsm_cycles;
			#ifdef DEBUG_X
			printf("[%04d]DS18B20 Temp: %.2f°C\n", now(), temperature / 100.0);
			#endif
		}	// DB18B20
		#endif

		#ifdef RFLAG_DS18B20_RISCV
		if ((flag & RFLAG_DS18B20_RISCV) != 0){	//ds18b20-Ergebnis
			ds18b20_result_t res = ds18b20_get_result();
			add_entry_I32(&pl, dt_ds18b20, 0, res.status, res.temperature);
			#ifdef DEBUG_X
			printf("[%04d]DS18B20 Temp: %.2f°C | Status: %d\n", now(), res.temperature / 100.0, res.status);
			#endif
		}	// DB18B20
		#endif

		#ifdef RFLAG_AM2302_FSM
		if ((flag & RFLAG_AM2302_FSM) != 0){	// DHT21
			int32_t temperature = (int32_t)ulp_temperature & 0xFFFF;	//10 * n°C
			uint32_t humidity   = ulp_humidity & 0xFFFF;	//10 * n%
			int8_t crc_check = ((temperature >> 8 ) + (temperature & 0xFF) +
				    (humidity >> 8 ) + (humidity & 0xFF)) -
   				    (ulp_crc8_value & 0xFF);
			rtc_fsm_cycles += ulp_cycles & 0xFFFF;	//Gesamtzähler ulp-cycles
			if (crc_check == 0) {
				ulp_cycles = 0;	//Rücksetzen f. nächsten Maincyle
				add_entry_I32(&pl, dt_am2302, 0, crc_check, temperature);
				add_entry_I32(&pl, dt_am2302, 1, crc_check, humidity);
				pl.man.ulp_cycles = rtc_fsm_cycles;
				#ifdef DEBUG_X
					printf("[%04d]AM2302 Temp: %.1f°C | Humi: %.1f%% | cyc: %d\n",
							now(), temperature / 10.0, humidity / 10.0, rtc_fsm_cycles);
				#endif
			} else {
				printf("AM2302 CRC-Error: %d\n", crc_check);
			}
		}	// DHT21
		#endif

		#ifdef RFLAG_BME280
		if ((flag & RFLAG_BME280) != 0) {
			bme280_result_t res = bme280_i2c_get_result();
			if (res.status == 0) {
				add_entry_I32(&pl, dt_bme280, 0, res.status, res.pressure);
				add_entry_I32(&pl, dt_bme280, 1, res.status, res.temperature);
				add_entry_I32(&pl, dt_bme280, 2, res.status, res.humidity);

				#ifdef DEBUG_X
					printf("[%04d]Press: %.2fhPa | Temp: %.2f°C | Humi: %.2f%% | Status: %d\n", now(),
							res.pressure/100000.0, res.temperature/1000.0, res.humidity/1000.0, res.status);
				#endif
			} else {
				add_entry_I32(&pl, dt_bme280, 0, 1, 0);	//Fehlermeldung Status = 1, Pressure = 0
				printf ("Fehler BME280\n");
			}
		} //bme280
		#endif

		#ifdef RFLAG_SHTC3
		if ((flag & RFLAG_SHTC3) != 0) {
			shtc3_result_t res = shtc3_i2c_get_result();
			if (res.status == 0) {
				printf("[%04d] SHTC3 - Temp: %.2f°C | Humi: %.2f%% | Status: %d\n", now(),
						res.temperature/100.0, res.humidity/100.0, res.status);
			} else {
				printf ("Fehler SHTC3\n");
			}
		}
		#endif //shtc3

		#ifdef RFLAG_SHT31
		if ((flag & RFLAG_SHT31) != 0) {
			int32_t temperature = (int32_t)ulp_sht31_temp;
			uint32_t humidity = ulp_sht31_humi;
			if (ulp_sht31_err == 0) {
				add_entry_I32(&pl, dt_sht3x, 0, 0, temperature);
				add_entry_I32(&pl, dt_sht3x, 1, 0, humidity);
				#ifdef DEBUG_X
				printf("[%04d] SHT31 - Temp: %.2f°C | Humi: %.2f%%\n",
						now(), temperature / 100.0, humidity / 100.0);
				#endif
			} else {
				printf ("Fehler SHT31: %d\n", ulp_sht31_err);
			}
		}
		#endif	//sht31

		#ifdef RFLAG_SHT31_RISCV
		if ((flag & RFLAG_SHT31_RISCV) != 0){
			int32_t temperature = (int32_t)ulp_sht31_temp;
			uint32_t humidity = ulp_sht31_humi;
			int8_t  status = (int8_t) ulp_sht31_err;
			add_entry_I32(&pl, dt_sht3x, 0, 0, temperature);
			add_entry_I32(&pl, dt_sht3x, 1, 0, humidity);
			pl.man.ulp_cycles = ulp_sht31_cycles;
			#ifdef DEBUG_X
			if (status == 0) {
				printf("ulp-cycle: %d\n", ulp_sht31_cycles);
				printf("[%04d] SHT31 - Temp: %.2f°C | Humi: %.2f%%\n",
						now(), temperature / 100.0, humidity / 100.0);
			} else {
				printf ("Fehler SHT31: %d\n", status);
			}
			#endif
		}
		#endif // sht31_ulp

		#ifdef RFLAG_BME280_RISCV
		if ((flag & RFLAG_BME280_RISCV) != 0){
			if (ulp_bme280_status == 0) {
				add_entry_I32(&pl, dt_bme280, 0, 0, ulp_bme280_pressure);
				add_entry_I32(&pl, dt_bme280, 1, 0, ulp_bme280_temperature);
				add_entry_I32(&pl, dt_bme280, 2, 0, ulp_bme280_humidity);
				pl.man.ulp_cycles = ulp_bme280_cycles;

				#ifdef DEBUG_X
				printf("Chip ID  : 0x%.2x\n", ulp_bme280_chip_id);
				printf("t-acquis : %.dms\n", ulp_bme280_acquisition_time_ms);
				printf("ulp-cycle: %d\n", ulp_bme280_cycles);
				printf("[%04d]Press: %.2fhPa | Temp: %.2f°C | Humi: %.2f%%\n", now(),
						ulp_bme280_pressure/100.0, ulp_bme280_temperature/100.0, ulp_bme280_humidity/1000.0);
				#endif
			} else {
				add_entry_I32(&pl, dt_bme280, 0, 1, 0);	//Fehlermeldung Status = 1, Pressure = 0
				printf ("Fehler BME280: %d\n", ulp_bme280_status);
				printf("Chip ID: 0x%.2x\n", ulp_bme280_chip_id);
			}
		}

		#endif
		// .....

		flags &= ~flag;
		if (flags == 0) break;
	}

	if (flags != 0)  { //Funktionsfehler
		add_entry_I32(&pl, dt_res_err, 0, 0, flags);	//Ergebnis fehlt
	}

	//Send Data to GW
	send_data_frame(&pl, pl.ix + sizeof(pl.man));

	#ifdef USE_ULP_RISCV
	//IDF v4.4 -> Domain Periph muss bei Verwendung riscv ausgeschaltet sein (default)
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
	#endif

	wiog_wifi_sensor_goto_sleep(WAKEUP_SOURCE);

	#ifdef USE_STEPUP_CTRL
	//Stepup-regler vor DeepSleep heruntertasten
   	rtc_gpio_set_level(pwr_ctrl, 0);
	#endif

   	printf("[%04d]Goto DeepSleep for %.3fs\n", now(), rtc_interval_ms / 1000.0);
	esp_deep_sleep_start();

	//----------------------------------------------------------------

}


