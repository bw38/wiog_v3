/*
 * ubat.h
 * Modul - Batteriespannungsmessung
 * ESP32S2
 */

#ifndef MAIN_UBAT_H_
#define MAIN_UBAT_H_


#include "esp_adc_cal.h"	//set COMPONENT_REQUIRES  esp_adc_cal in CMakeLists.txt

//extern SemaphoreHandle_t UBat_Semaphore;

extern esp_adc_cal_value_t vref_cal_type;	//Calibrierungstyp
//int Ubat_ADC_mV;							//Messergebnis in mV

//Ports initialisieren und Task starten
extern void ubat_init(adc1_channel_t ch, gpio_num_t in_gnd, uint32_t spl, uint32_t vref);
extern void ubat_start(QueueHandle_t hQ, uint32_t flag);
extern uint32_t ubat_get_result();

#endif /* MAIN_UBAT_H_ */
