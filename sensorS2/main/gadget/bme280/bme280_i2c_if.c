/*
 * bme280_i2c_if.c
 *
 *  Created on: 26.04.2019
 *      Author: joerg

  ESP32-I2C - Beispielcode:
  https://github.com/espressif/esp-idf/blob/a20d02b7f196c407bc9f39b781e31a0a4f665968/examples/peripherals/i2c/i2c_self_test/main/i2c_example_main.c
  Bosch-Treiber: file bme280.c /  2020-03-28 / v3.5.0
  https://github.com/BoschSensortec/BME280_driver
*/


#include "bme280_i2c_if.h"

#define DEBUG_I2C_NO

//interne Varibalen
RTC_DATA_ATTR static struct bme280_dev bme280_device;
RTC_DATA_ATTR static uint8_t dev_addr = 0;	//0x76 | 0x77
struct bme280_data bme280_comp_data;


i2c_port_t i2c_mport;
static uint32_t rflag = 0;
static QueueHandle_t hResponseQueue;

RTC_DATA_ATTR static int8_t rtc_bme280_init_result;

//Prototypen
void get_data_bme280_i2c_task(void * pvParameters);


//Hilfsfunktionen --------------------------------------------------
#ifdef DEBUG_I2C
static void print_now(){
	printf("[%.4d]", (int)esp_timer_get_time() / 1000);
}
#endif


// I2C - Anpassung an BME280.h --------------------------------

//only called once in coldstart (2ms)
void bme280_delay_10ms_i2c(uint32_t period, void *intf_ptr) {
#ifdef DEBUG_I2C
	print_now();
	printf("delay_us: %d\n", period);
#endif
	vTaskDelay(1);   //Tick-Rate in MenuConfig 10ms !!!
}

int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if (len == 0) return ESP_OK;

    uint8_t dev_addr = *((uint8_t *)intf_ptr);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg_addr, 1);

    i2c_master_start(cmd);	//Repeated Start
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, 1);

    if (len > 1) i2c_master_read(cmd, reg_data, len - 1, 0);
    i2c_master_read_byte(cmd, reg_data + len - 1, 1);

    i2c_master_stop(cmd);
    int8_t res = i2c_master_cmd_begin(i2c_mport, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

#ifdef DEBUG_I2C
    print_now();
    printf("Rd** Dev: %.2X | Reg: %.2X | Len: %d |Data: ", dev_addr, reg_addr,  len);
	for (int i=0; i<len; i++) printf(" %.2X", reg_data[i]);
	printf("\n");
#endif
	return res;
}

int8_t bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *((uint8_t *)intf_ptr);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, 1);
    for (int i=0; i<len; i++)
    {
    	i2c_master_write_byte(cmd, reg_addr + i, 1);
    	i2c_master_write_byte(cmd, reg_data[i], 1);
    }
    i2c_master_stop(cmd);
    int8_t ret = i2c_master_cmd_begin(i2c_mport, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

#ifdef DEBUG_I2C
    print_now();
	printf("Wr** Dev: %.2X | Reg: %.2X | Len: %d |Data: ", dev_addr, reg_addr,  len);
	for (int i=0; i<len; i++) printf(" %.2X", reg_data[i]);
	printf("\n");
#endif

    return ret;
}
// --------------------------------------------------------------------------------------------

//Schnittstelle initialisieren
//erforderlich nach jedem boot bzw wakeup
//wu == true -> wakeup aus Tiefschlaf
esp_err_t bme280_i2c_init(i2c_port_t _i2c_mport, bool wu)
{
	i2c_mport = _i2c_mport;
    //Chip wird nach Kaltstart oder vorherg. Fehler initialisiert
    if (! wu) rtc_bme280_init_result = -1;
	if (rtc_bme280_init_result != 0) {
		dev_addr = BME280_I2C_ADDR_PRIM;	//SDO 10k Pulldown auf Sensorboard
		bme280_device.intf_ptr = &dev_addr;
		bme280_device.intf  = BME280_I2C_INTF;
		bme280_device.read  = bme280_i2c_read;
		bme280_device.write = bme280_i2c_write;
		bme280_device.delay_us = bme280_delay_10ms_i2c;
		rtc_bme280_init_result = bme280_init(&bme280_device); // --> bme280.h

		//safe side -> SW-Reset
		const uint8_t com_res = BME280_SOFT_RESET_COMMAND;
		bme280_i2c_write(BME280_RESET_ADDR, &com_res, 1, &bme280_device);
		bme280_delay_10ms_i2c(1, NULL); // > 2ms PowerOn-Reset

		#ifdef DEBUG_I2C
		printf("BME-Init: %d\n", rtc_bme280_init_result);
		#endif
	}
	return rtc_bme280_init_result;
}

void bme280_i2c_start(QueueHandle_t hQ, uint32_t flag)
{
	hResponseQueue = hQ;
	rflag = flag;
	//Messvorgang starten
	if (rtc_bme280_init_result == 0)
		xTaskCreate(get_data_bme280_i2c_task, "bme280", 2048, NULL, 3, NULL);
}


//Task - Messung anstoßen, nach xxms Daten lesen
void get_data_bme280_i2c_task(void *pvParameters)
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
	vTaskDelay(50 / portTICK_PERIOD_MS);
	rslt = bme280_get_sensor_data(BME280_ALL, &bme280_comp_data, &bme280_device) | rslt;

	rslt = bme280_set_sensor_mode(BME280_SLEEP_MODE, &bme280_device) | rslt;

	if (rslt != 0) rtc_bme280_init_result = -1; //beim nächsten mal neu initialisieren

	//Fertigmeldung an main
	xQueueSend(hResponseQueue, &rflag, portMAX_DELAY);
	vTaskDelete(NULL);
}

bme280_result_t bme280_i2c_get_result() {
	//Messergebnisse
	bme280_result_t res;
	res.pressure = bme280_comp_data.pressure * 1000;		//pa * 1000
	res.temperature = bme280_comp_data.temperature * 1000;	//°C * 1000
	res.humidity = bme280_comp_data.humidity * 1000;		//%  * 1000
	if ((res.pressure == 0) || (res.humidity == 0))
		rtc_bme280_init_result = 9;
	res.status = rtc_bme280_init_result;
	return res;
}
