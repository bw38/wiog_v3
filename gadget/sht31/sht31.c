/*
 * shtc3_i2c_if.c
 *
 *  Created on: 30.09.2021
 *      Author: joerg
 */


#include "sht31.h"

#define DEBUG_I2C_NO


i2c_port_t i2c_mport;
static uint32_t rflag = 0;
static QueueHandle_t hResponseQueue;

#define SHT31_I2C_ADDR_L	0x44	//Herstellervorgabe
#define SHT31_I2C_ADDR_H	0x45

#define SHT31_SW_RESET 		0x30A2
#define SHT31_READ_STATUS	0xF32D
#define SHT31_CLEAR_STATUS	0x3041

//Start-Modes - Single Shot - ClockStretching disabled
#define SHT31_MODE_LOW		0x2416
#define SHT31_MODE_MED		0x240B
#define SHT31_MODE_HIGH		0x2400

#define SHT31_HEATER_EN		0x306D
#define SHT31_HEATER_DIS	0x3066

//#define SHT31_SLEEP			0xB098
//#define SHT31_WAKEUP		0x3517

static uint8_t data[16];	//RD- / WR-Buffer
sht31_result_t sht31_result;

RTC_DATA_ATTR static int8_t rtc_sht31_init_result;

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


static int8_t dev_i2c_read(uint8_t *reg_data, uint32_t len)
{
    if (len == 0) return ESP_OK;

    uint8_t dev_addr = SHT31_I2C_ADDR_L;
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

static int8_t dev_i2c_write_word(uint16_t dataw)
{
    uint8_t dev_addr = SHT31_I2C_ADDR_L;
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
esp_err_t sht31_i2c_init(i2c_port_t _i2c_mport, bool wu)
{
	esp_err_t err_dev = ESP_OK;
	i2c_mport = _i2c_mport;
    //Chip wird nach Kaltstart oder vorherg. Fehler initialisiert
    if (! wu) rtc_sht31_init_result = -1;
	if (rtc_sht31_init_result != 0) {
		//SoftReset
		dev_i2c_write_word(SHT31_SW_RESET);
		dev_i2c_write_word(SHT31_CLEAR_STATUS);
		vTaskDelay(1);
		//Read Status-Reg
		dev_i2c_write_word(SHT31_READ_STATUS);
		dev_i2c_read (data, 3);
		uint16_t status = data[0]*256 + data[1];
		rtc_sht31_init_result = 0;
		if (checkCRC(status, data[2]) != 0) {
			err_dev = ESP_FAIL;
			rtc_sht31_init_result = -1;
		}
		#ifdef DEBUG_I2C
		printf("SHT31-Init: %d\n", err_dev);
		#endif
	}
	return err_dev;
}




//Task - Messung anstoßen, nach xxms Daten lesen
void get_data_dev_i2c_task(void *pvParameters)
{
	dev_i2c_write_word(SHT31_MODE_MED);
	vTaskDelay(1);
	dev_i2c_read (data, 6);

	uint16_t raw_temp = data[0]*256 + data[1];
	uint16_t raw_humi = data[3]*256 + data[4];

	if ((checkCRC(raw_humi, data[5])) | (checkCRC(raw_temp, data[2]) != 0)) {
		sht31_result.status = -1;
		sht31_result.humidity = 0;
		sht31_result.temperature = 0;
		rtc_sht31_init_result = -1;
	} else {
		sht31_result.status = 0;
		sht31_result.humidity = (100 * raw_humi / 65535.0) * 100;
		sht31_result.temperature = ((175 * raw_temp / 65535.0) - 45) * 100;
	}

	//Fertigmeldung an main
	xQueueSend(hResponseQueue, &rflag, portMAX_DELAY);
	vTaskDelete(NULL);
}

void sht31_i2c_start(QueueHandle_t hQ, uint32_t flag)
{
	hResponseQueue = hQ;
	rflag = flag;
	//Messvorgang starten
//	if (rtc_shtc3_init_result == 0)
		xTaskCreate(get_data_dev_i2c_task, "shtc3", 2048, NULL, 3, NULL);
}

sht31_result_t sht31_i2c_get_result() {
	//Messergebnisse
	sht31_result_t res;
	res.temperature = sht31_result.temperature; //°C
	res.humidity = sht31_result.humidity;		//%
	res.status = sht31_result.status;
	return res;
}
