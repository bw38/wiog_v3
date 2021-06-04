/*
 * bme280_spi_if.c
 *
 *  Created on: 22.04.2019
 *      Author: joerg
 */

// *******************************************************************************************

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#include "driver/spi_master.h"

#include "bme280_spi_if.h"
#include "../sensor_main.h"

#define DEBUG_SPIx


//interne Varibalen
spi_device_handle_t h_spi_master;
RTC_DATA_ATTR static struct bme280_dev bme280_device;

struct bme280_data bme280_comp_data;
SemaphoreHandle_t bme280_Semaphore;

RTC_DATA_ATTR static int8_t rtc_bme280_init_result;

//Prototypen
int8_t bme280_spi_init(void);
void get_data_bme280_spi_task(void * pvParameters);


//SPI -Interface ESP32

//Init VSPI, KEIN DMA
esp_err_t bme280_spi_master_init(bool wu)
{
	esp_err_t err_spi;
    spi_device_handle_t h_spi;
    spi_bus_config_t buscfg = {
        .miso_io_num = BME280_SPI_MISO,
        .mosi_io_num = BME280_SPI_MOSI,
        .sclk_io_num = BME280_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num= - 1,
        .max_transfer_sz = 128
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1*1000*1000,         //Clock out at 10 MHz
        .mode = 0,                              //SPI mode 0
        .spics_io_num = BME280_SPI_CS,                 //CS pin
        .queue_size = 1,                        //transactions at a time
        .pre_cb = NULL,                         //Specify pre-transfer callback
        .post_cb = NULL,						//post-callback
		.command_bits = 0,						//no BME-Commands
		.address_bits = 8						//BME-Adress-width
    };

    //Initialize the SPI bus
    err_spi = spi_bus_initialize(VSPI_HOST, &buscfg, 0);    // kein DMA
    ESP_ERROR_CHECK(err_spi);
    //Attach the BME to the SPI bus
    err_spi = spi_bus_add_device(VSPI_HOST, &devcfg, &h_spi);
    ESP_ERROR_CHECK(err_spi);
	h_spi_master = h_spi;

    //Chip wird nur nach Kaltstart initialisiert
    if (! wu) rtc_bme280_init_result = -1;
	if (rtc_bme280_init_result != 0) {
		rtc_bme280_init_result = bme280_spi_init();
		#ifdef DEBUG_SPIx
			printf("BME-Init: %d | %d\n", rtc_bme280_init_result, err_spi);
		#endif
	}

	//Messvorgang starten
	bme280_Semaphore = xSemaphoreCreateBinary();	//Vorbereitend für Messende
	if (rtc_bme280_init_result == 0)
		xTaskCreate(get_data_bme280_spi_task, "bme280", 2048, NULL, 3, NULL);

	return rtc_bme280_init_result | err_spi;
}



esp_err_t spi_writeBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data) {
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.cmd = 0;
    transaction.addr = regAddr;
    transaction.length = length * 8;
    transaction.rxlength = 0;
    transaction.user = NULL;
    transaction.tx_buffer = data;
    transaction.rx_buffer = NULL;
    esp_err_t err = spi_device_polling_transmit(handle, &transaction);

    return err;
}


esp_err_t spi_readBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data) {
    if(length == 0) return ESP_ERR_INVALID_SIZE;
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.cmd = 0;
    transaction.addr = regAddr;
    transaction.length = length * 8 + 8;
    transaction.rxlength = length * 8;
    transaction.user = NULL;
    transaction.tx_buffer = NULL;
    transaction.rx_buffer = data;
    esp_err_t err = spi_device_polling_transmit(handle, &transaction);

    return err;
}

// SPI - Anpassung an BME280.h --------------------------------

void bme280_delay_ms(uint32_t period)
{
    vTaskDelay(period);    //Tick-Rate in MenuConfig 10ms !!!
}

int8_t bme280_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	int8_t res = spi_readBytes(h_spi_master, reg_addr, len, reg_data);

#ifdef DEBUG_SPI
	printf("Rd** addr: %.2X | Len: %d |Data: ", reg_addr,  len);
	for (int i=0; i<len; i++) printf(" %.2X", reg_data[i]);
	printf("\n");
#endif
	return res;
}

int8_t bme280_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
#ifdef DEBUG_SPI
	printf("Wr** addr: %.2X | Len: %d |Data: ", reg_addr,  len);
	for (int i=0; i<len; i++) printf(" %.2X", reg_data[i]);
	printf("\n");
#endif

    return spi_writeBytes(h_spi_master, reg_addr, len, reg_data);
}


int8_t bme280_spi_init(void)
{
	bme280_device.dev_id = 0;
	bme280_device.intf  = BME280_SPI_INTF;
	bme280_device.read  = bme280_spi_read;
	bme280_device.write = bme280_spi_write;
	bme280_device.delay_ms = bme280_delay_ms;
	uint8_t res = bme280_init(&bme280_device); // --> bme280.h
	return res;
}

// -------------------------------------------------

void get_data_bme280_spi_task(void * pvParameters)
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

	bme280_Semaphore = xSemaphoreCreateBinary();
//printf("[%04d] Start\n", (int)esp_timer_get_time() / 1000);
	vTaskDelay(50/portTICK_PERIOD_MS);	//44ms gemessen
//printf("[%04d] Ready\n", (int)esp_timer_get_time() / 1000);
	rslt = bme280_get_sensor_data(BME280_ALL, &bme280_comp_data, &bme280_device) | rslt;

	#ifdef DEBUG_X
		printf("rslt= %d -> temp %d, p %d, hum %d\r\n", rslt, bme280_comp_data.temperature,  bme280_comp_data.pressure,  bme280_comp_data.humidity);
	#endif

	xSemaphoreGive(bme280_Semaphore);
	vTaskDelete(NULL);
}


//Messwerte in Payload (main) eintragen
void bme280_add_entries() {
	uint8_t res;
	if( xSemaphoreTake(bme280_Semaphore, 300 * MS ) == pdTRUE ) {
		res = 0;
		//Reihenfolge der Messwerte zur Berechnung des red Luftdruck
		union data_entry_t debmet = {	//Temperatur
			.type = BME280__TEMP,
			.status = res,
			.value = bme280_comp_data.temperature
		};
		Add_Entry(debmet);

		union data_entry_t debmeh = {	//Luftfeuchtigkeit
			.type = BME280__HUMI,
			.status = res,
			.value = bme280_comp_data.humidity
		};
		Add_Entry(debmeh);

		union data_entry_t debmep = {	//Luftdruck
			.type = BME280__PRESS,
			.status = res,
			.value = bme280_comp_data.pressure
		};
		Add_Entry(debmep);

	}
}

