/*
 * bme280_i2c_if.c
 *
 *  Created on: 26.04.2019
 *      Author: joerg
 */

//ESP32-I2C - Beispielcode:
//https://github.com/espressif/esp-idf/blob/a20d02b7f196c407bc9f39b781e31a0a4f665968/examples/peripherals/i2c/i2c_self_test/main/i2c_example_main.c


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver/rtc_io.h"

#include "bme280_i2c_if.h"
#include "bme280_defs.h"

#include "../../wiog_include/wiog_data.h"

#define DEBUG_I2C_NO

#define I2C_MASTER_FREQ_HZ	 250000	//Frequenz in Hz, max 1MHz

//interne Varibalen
RTC_DATA_ATTR static struct bme280_dev bme280_device;
struct bme280_data bme280_comp_data;
//SemaphoreHandle_t bme280_Semaphore;

i2c_port_t i2c_mport;
static uint32_t rflag = 0;

//Prototypen
int8_t bme280_i2c_init(void);
void get_data_bme280_i2c_task(void * pvParameters);


//I2C -Interface ESP32


//Schnittstelle initialisieren
//erforderlich nach jedem boot bzw wakeup
//wu == true -> wakeup aus Tiefschlaf
esp_err_t bme280_i2c_master_init(i2c_port_t i2c_master_port, int sda_io_num, int scl_io_num, bool wu) {
	i2c_mport = i2c_master_port;
    i2c_config_t conf;
    conf.clk_flags = 0; // ab IDF > v4.1
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_io_num;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;	//10k pullup auf Sensorboard
    conf.scl_io_num = scl_io_num;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;	//10k Pullup auf Sensorboard
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    esp_err_t err_i2c = i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);

    //Chip wird nur nach Kaltstart initialisiert
    if (! wu) rtc_bme280_init_result = -1;
	if (rtc_bme280_init_result != 0) {
		rtc_bme280_init_result = bme280_i2c_init();
		#ifdef DEBUG_I2C
			printf("BME-Init: %d | %d\n", rtc_bme280_init_result, err_i2c);
		#endif
	}
	return err_i2c | rtc_bme280_init_result;
}

void bme280_i2c_start(uint32_t flag) {
	rflag = flag;
	//Messvorgang starten
	if (rtc_bme280_init_result == 0)
		xTaskCreate(get_data_bme280_i2c_task, "bme280", 2048, NULL, 3, NULL);
}


bme280_result_t bme280_i2c_get_result() {
	//Messergebnisse
	bme280_result_t res;
	res.pressure = bme280_comp_data.pressure;
	res.temperature = bme280_comp_data.temperature;
	res.humidity = bme280_comp_data.humidity;
	res.status = rtc_bme280_init_result;
	return res;
}

// I2C - Anpassung an BME280.h --------------------------------

void bme280_delay_10ms_i2c(uint32_t period)
{
    vTaskDelay(period);   //Tick-Rate in MenuConfig 10ms !!!
}

int8_t bme280_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    if (len == 0) return ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg_addr, 1);

    i2c_master_start(cmd);	//Repeated Start
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_READ, 1);

    if (len > 1) i2c_master_read(cmd, reg_data, len - 1, 0);
    i2c_master_read_byte(cmd, reg_data + len - 1, 1);

    i2c_master_stop(cmd);
    esp_err_t res = i2c_master_cmd_begin(i2c_mport, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

#ifdef DEBUG_I2C
	printf("Rd** addr: %.2X | Len: %d |Data: ", reg_addr,  len);
	for (int i=0; i<len; i++) printf(" %.2X", reg_data[i]);
	printf("\n");
#endif
	return res;
}

int8_t bme280_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
#ifdef DEBUG_I2C
	printf("Wr** addr: %.2X | Len: %d |Data: ", reg_addr,  len);
	for (int i=0; i<len; i++) printf(" %.2X", reg_data[i]);
	printf("\n");
#endif

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, 1);
    for (int i=0; i<len; i++)
    {
    	i2c_master_write_byte(cmd, reg_addr++, 1);
    	i2c_master_write_byte(cmd, reg_data[i], 1);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_mport, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


//BME280 initialisieren, nur nach reset, nicht nach wakeup
int8_t bme280_i2c_init(void)
{
	bme280_device.dev_id = 0x76;	//SDO 10k Pulldown auf Sensorboard
	bme280_device.intf  = BME280_I2C_INTF;
	bme280_device.read  = bme280_i2c_read;
	bme280_device.write = bme280_i2c_write;
	bme280_device.delay_ms = bme280_delay_10ms_i2c;
	uint8_t res = bme280_init(&bme280_device); // --> bme280.h
	return res;
}

//Task - Messung anstoßen, nach xxms Daten lesen, Semaphore freigeben
void get_data_bme280_i2c_task(void * pvParameters)
{
	bme280_device.settings.osr_h = BME280_OVERSAMPLING_2X;
	bme280_device.settings.osr_p = BME280_OVERSAMPLING_8X;
	bme280_device.settings.osr_t = BME280_OVERSAMPLING_2X;
	bme280_device.settings.filter = BME280_FILTER_COEFF_OFF;

	uint8_t settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
	//Oversampling und Filter setzen
	int8_t rslt = bme280_set_sensor_settings(settings_sel, &bme280_device);
	//Messung anstoßen
	rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &bme280_device) | rslt;
//	bme280_Semaphore = xSemaphoreCreateBinary();
//printf("[%04d] Start\n", (int)esp_timer_get_time() / 1000);
		vTaskDelay(50 * MS);	//44ms gemessen
//printf("[%04d] Ready\n", (int)esp_timer_get_time() / 1000);;
	rslt = bme280_get_sensor_data(BME280_ALL, &bme280_comp_data, &bme280_device) | rslt;

	rslt = bme280_set_sensor_mode(BME280_SLEEP_MODE, &bme280_device) | rslt;

	if (rslt != 0) rtc_bme280_init_result = -1; //beim nächsten mal neu initialisieren

	//Fertigmeldung an main
	xQueueSend(measure_response_queue, &rflag, portMAX_DELAY);
	vTaskDelete(NULL);
}


