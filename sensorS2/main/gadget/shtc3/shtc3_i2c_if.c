/*
 * shtc3_i2c_if.c
 *
 *  Created on: 30.09.2021
 *      Author: joerg
 */


#include "shtc3_i2c_if.h"

#define DEBUG_I2C_NO


i2c_port_t i2c_mport;
static uint32_t rflag = 0;
static QueueHandle_t hResponseQueue;


#define SHTC3_I2C_ADDR		0x70	//Herstellervorgabe

#define SHTC3_SW_RESET 		0x805D
#define SHTC3_READ_ID		0xEFC8
#define SHTC3_MEAS_LOWPWR	0x609C
#define SHTC3_MEAS_NORMAL	0x7866
#define SHTC3_SLEEP			0xB098
#define SHTC3_WAKEUP		0x3517

static uint8_t data[16];	//RD- / WR-Buffer
shtc3_result_t shtc3_result;

RTC_DATA_ATTR static int8_t rtc_shtc3_init_result = -1;

//Hilfsfunktionen --------------------------------------------------
#ifdef DEBUG_I2C
static void print_now(){
	printf("[%.4d]", (int)esp_timer_get_time() / 1000);
}
#endif

static uint8_t checkCRC(uint16_t packet, uint8_t cs)
{
	uint8_t upper = packet >> 8;
	uint8_t lower = packet & 0x00FF;
	uint8_t data[2] = {upper, lower};
	uint8_t crc = 0xFF;
	uint8_t poly = 0x31;

	for (uint8_t indi = 0; indi < 2; indi++) {
		crc ^= data[indi];
		for (uint8_t indj = 0; indj < 8; indj++) {
			if (crc & 0x80)	{
				crc = (uint8_t)((crc << 1) ^ poly);
			} else {
				crc <<= 1;
			}
		}
	}
	return (cs ^ crc);
}


static int8_t shtc3_i2c_read(uint8_t *reg_data, uint32_t len)
{
    if (len == 0) return ESP_OK;

    uint8_t dev_addr = SHTC3_I2C_ADDR;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, 1);

    if (len > 1) i2c_master_read(cmd, reg_data, len - 1, 0);
    i2c_master_read_byte(cmd, reg_data + len - 1, 1);

    i2c_master_stop(cmd);
    int8_t res = i2c_master_cmd_begin(i2c_mport, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

#ifdef DEBUG_I2C
    print_now();
    printf("Rd** Dev: %.2X | Len: %d |Data: ", dev_addr, len);
	for (int i=0; i<len; i++) printf(" %.2X", reg_data[i]);
	printf("\n");
#endif
	return res;
}

static int8_t shtc3_i2c_write_word(uint16_t dataw)
{
    uint8_t dev_addr = SHTC3_I2C_ADDR;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, 1);

    i2c_master_write_byte(cmd, (dataw >> 8) & 0xFF, 1);
    i2c_master_write_byte(cmd, dataw & 0xFF, 1);

    i2c_master_stop(cmd);
    int8_t ret = i2c_master_cmd_begin(i2c_mport, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

#ifdef DEBUG_I2C
    print_now();
	printf("Wr** Dev: %.2X | %.4X\n", dev_addr, dataw);
#endif

    return ret;
}
// --------------------------------------------------------------------------------------------



//Schnittstelle initialisieren
//erforderlich nach jedem boot bzw wakeup
//wu == true -> wakeup aus Tiefschlaf
esp_err_t shtc3_i2c_init(i2c_port_t _i2c_mport, bool wu)
{
	esp_err_t err_dev = ESP_OK;
	i2c_mport = _i2c_mport;
    //Chip wird nach Kaltstart oder vorherg. Fehler initialisiert
    if (! wu) rtc_shtc3_init_result = -1;
	if (rtc_shtc3_init_result != 0) {
		//SoftReset
		shtc3_i2c_write_word(SHTC3_SW_RESET);
		vTaskDelay(1);
		shtc3_i2c_write_word(SHTC3_WAKEUP);
		vTaskDelay(1);
		//Read ID-Reg
		shtc3_i2c_write_word(SHTC3_READ_ID);
		shtc3_i2c_read (data, 3);
		uint16_t ID = data[0]*256 + data[1];
		rtc_shtc3_init_result = 0;
		if ((checkCRC(ID, data[2]) != 0) ||	((ID & 0b0000100000111111) != 0b0000100000000111)) {
			err_dev = ESP_FAIL;
			rtc_shtc3_init_result = -1;
		}
		#ifdef DEBUG_I2C
		printf("SHTC3-Init: %d\n", err_dev);
		#endif
	}
	return err_dev;
}




//Task - Messung anstoßen, nach xxms Daten lesen
void get_data_shtc3_i2c_task(void *pvParameters)
{
	shtc3_i2c_write_word(SHTC3_WAKEUP);
	vTaskDelay(1);
	shtc3_i2c_write_word(SHTC3_MEAS_NORMAL);
	vTaskDelay(20 / portTICK_PERIOD_MS);
	shtc3_i2c_read (data, 6);
	shtc3_i2c_write_word(SHTC3_SLEEP);

	uint16_t raw_temp = data[0]*256 + data[1];
	uint16_t raw_humi = data[3]*256 + data[4];
	if ((checkCRC(raw_humi, data[5])) | (checkCRC(raw_temp, data[2]) == 0)) {
		shtc3_result.status = 0;
		shtc3_result.humidity = (100 * raw_humi / 65535.0) * 100;
		shtc3_result.temperature = ((175 * raw_temp / 65535.0) - 45) * 100;
	} else {
		shtc3_result.status = -1;
		shtc3_result.humidity = 0;
		shtc3_result.temperature = 0;
		rtc_shtc3_init_result = -1;
	}

	//Fertigmeldung an main
	xQueueSend(hResponseQueue, &rflag, portMAX_DELAY);
	vTaskDelete(NULL);
}

void shtc3_i2c_start(QueueHandle_t hQ, uint32_t flag)
{
	hResponseQueue = hQ;
	rflag = flag;
	//Messvorgang starten
//	if (rtc_shtc3_init_result == 0)
		xTaskCreate(get_data_shtc3_i2c_task, "shtc3", 2048, NULL, 3, NULL);
}

shtc3_result_t shtc3_i2c_get_result() {
	//Messergebnisse
	shtc3_result_t res;
	res.temperature = shtc3_result.temperature; //°C
	res.humidity = shtc3_result.humidity;		//%
	res.status = shtc3_result.status;
	return res;
}
