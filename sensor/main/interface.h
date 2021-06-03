/*
 * interface.h
 *
 *  Created on: 28.05.2021
 *      Author: joerg
 */

#ifndef MAIN_INTERFACE_H_
#define MAIN_INTERFACE_H_

//StepUp-Regler-Steuerung
#define GPIO_STEPUP_CTRL	32
#define RTC_IO_STEPUP_CTRL	9

//OneWirePort DS18B20
#define GPIO_DS18B20_OWP	27
#define RTC_IO_DS18B20_OWP	17

#define UBAT_ADC_CHN		ADC1_CHANNEL_7		//Messeingang	GPIO35
//#define UBAT_DIV_GND		GPIO_NUM_33			//Spannungsteiler während Messung nach unten ziehen
#define UBAT_DIV_GND		-1	 				// -1 -> ohne Spannungsteiler
#define DIVIDER_SCALING		39200/8200			//Einzelwiderstände gemessen

#endif /* MAIN_INTERFACE_H_ */
