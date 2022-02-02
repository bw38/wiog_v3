
/*
 * ds18b20.c
 *
 *  Created on: 03.02.2020
 *      Author: joerg
 */

!!!!! nicht mehr benötigt, wird in Sensor_main.c direkt erledigt !!!!

#include "freertos/FreeRTOS.h"

#include "driver/rtc_io.h"
#include "../ds18b20_alt/ds18b20.h"
#include "ulp_main.h"



void ds18b20_init(gpio_num_t owp1){
	//DS18B20-Messung vorbereiten
	#ifdef RFLAG_DS18B20_FSM
	rtc_gpio_init(owp1);
	rtc_gpio_set_direction(owp1, RTC_GPIO_MODE_INPUT_ONLY);
	rtc_gpio_pullup_en(owp1);		//Dataport intern pullup
	rtc_gpio_pulldown_dis(owp1);
	rtc_gpio_hold_dis(owp1);
	#endif
	ulp_owp = owp1;
}


//Messung wurde bereits im ULP-Programm gestartet
//hier nur sofortige Rückgabe des Messwertes
void ds18b20_start(xQueueHandle hQ, uint32_t flag) {
	xQueueSend(hQ, &flag, portMAX_DELAY);
}

//Rückgabe Messergebnis
ds18b20_result_t ds18b20_get_result() {
	ds18b20_result_t res;
	res.temperature = (((int32_t)(ulp_temp_raw) & UINT16_MAX)*100) / 16.0;
	res.status = (uint8_t)ulp_crc_err;
	return res;
}
