/*
 * am2302.c
 *
 *  Created on: 03.02.2020
 *      Author: joerg
 */

#include "../am2302_alt/am2302.h"

#include "driver/rtc_io.h"
#include "../../wiog_include/wiog_data.h"
#include "ulp_main.h"

void am2302_init(gpio_num_t owp){
   	//DTH22-Messung vorbereiten
   	rtc_gpio_init(owp);
  	rtc_gpio_set_direction(owp, RTC_GPIO_MODE_INPUT_ONLY);
}

//nur Rückgabe der Werte
void am2302_start(uint32_t flag) {
	xQueueSend(measure_response_queue, &flag, portMAX_DELAY);
}


am2302_result_t am2302_get_result() {
	am2302_result_t res;
	res.temperature = ulp_temperature & 0xFFFF;
	res.humidity    = ulp_humidity & 0xFFFF;
	res.crc_check = ((res.temperature >> 8 ) + (res.temperature & 0xFF) +
					    (res.humidity >> 8 ) + (res.humidity & 0xFF)) -
	   				    (ulp_crc8_value & 0xFF);
	return res;
}

/*
void am2302_add_entries() {
	//Ergenis Luftfeuchtigkeit ULP-Programm
   	uint32_t temperature_dC = ulp_temperature & 0xFFFF;
   	uint32_t humidity_mil = ulp_humidity & 0xFFFF;

   	int8_t crc_check = ((temperature_dC >> 8 ) + (temperature_dC & 0xFF) + (humidity_mil >> 8 ) + (humidity_mil & 0xFF)) -
   				        (ulp_crc8_value & 0xFF);

   	//Temperature < 0°C
   	if (temperature_dC > 0x7FFF) { //neg. Wert
   		temperature_dC &= 0x7FFF;
   		temperature_dC *= -1;
   	}

   	//Datensatz Temperatur
	union data_entry_t det2 = {
		.type = TEMP_DC,
		.status = crc_check,
		.value = temperature_dC};
	Add_Entry(det2);

	//Datensatz Luftfeuchtigkeit
	union data_entry_t deh2 = {
		.type = HUMIDITY_MIL,
		.status = crc_check,
		.value = humidity_mil};
	Add_Entry(deh2);
}
*/



