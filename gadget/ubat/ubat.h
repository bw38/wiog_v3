/*
 * ubat.h
 * Modul - Batteriespannungsmessung
 */

#ifndef MAIN_UBAT_H_
#define MAIN_UBAT_H_

#include "esp_adc_cal.h"

//extern SemaphoreHandle_t UBat_Semaphore;

extern esp_adc_cal_value_t vref_cal_type;	//Calibrierungstyp
//int Ubat_ADC_mV;							//Messergebnis in mV

//Ports initialisieren und Task starten
extern void ubat_init(adc1_channel_t ch, gpio_num_t in_gnd, uint32_t spl);
extern void ubat_start(uint32_t flag);
extern uint32_t ubat_get_result();

extern void ubat_set_vref(uint32_t vref);

#endif /* MAIN_UBAT_H_ */
