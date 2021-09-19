/*
 * interface.h
 *
 *  Created on: 28.05.2021
 *      Author: joerg
 *
 *  DHT22 / AM2302 - Rohrmelder
 *
 *  UIDs
 *   - 60717
 */

#ifndef MAIN_INTERFACE_H_
#define MAIN_INTERFACE_H_

#define UID60717

#define AM2302_GPIO_OWP		26
#define AM2302_RTC_IO_OWP	7

//StepUp-Regler-Steuerung (bisher nicht genutzt)
#define GPIO_STEPUP_CTRL	32
#define RTC_IO_STEPUP_CTRL	9

#define UBAT_ADC_CHN		ADC1_CHANNEL_7		//Messeingang	GPIO35
#define UBAT_DIV_GND		GPIO_NUM_32			//Spannungsteiler während Messung nach unten ziehen
//#define UBAT_DIV_GND		-1	 				// -1 -> ohne Spannungsteiler
#define UBAT_DIVIDER		22000/8200			//Einzelwiderstände gemessen

//Chipabhängige Referenzspannung
#ifdef UID60717
	#define VREF			1125
#endif

#define LED_STATUS			GPIO_NUM_12			//Status - LED
#define LED_STATUS_REG		IO_MUX_GPIO12_REG	//MUX-Reg


#endif /* MAIN_INTERFACE_H_ */
