/*
 * ds18b20.c
 *
 *  Created on: 03.02.2020
 *      Author: joerg
 */


#include "driver/rtc_io.h"
#include "math.h"

#include "../../wiog_include/wiog_data.h"

#include "ds18b20.h"
//#include "../sensor_main.h"
#include "ulp_main.h"


void ds18b20_init(gpio_num_t owp1){
	//DS18B20-Messung vorbereiten
	rtc_gpio_init(owp1);
	rtc_gpio_set_direction(owp1, RTC_GPIO_MODE_INPUT_ONLY);
	rtc_gpio_pullup_en(owp1);		//Dataport intern pullup
	rtc_gpio_pulldown_dis(owp1);
	rtc_gpio_hold_dis(owp1);
}

//Messung wurde bereits im ULP-Programm gestartet
//hier nur sofortige Rückgabe des Messwertes
void ds18b20_start(uint32_t flag) {
	ds18b20_temperature = ((ulp_temp_raw & UINT16_MAX)*100) / 16.0;
	xQueueSend(measure_response_queue, &flag, portMAX_DELAY);
}

