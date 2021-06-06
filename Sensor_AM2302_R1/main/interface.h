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

#define AM2302_GPIO_OWP		26
#define AM2302_RTC_IO_OWP	7

#define UBAT_ADC_CHN		ADC1_CHANNEL_7		//Messeingang	GPIO35
#define UBAT_DIV_GND		GPIO_NUM_32			//Spannungsteiler während Messung nach unten ziehen
//#define UBAT_DIV_GND		-1	 				// -1 -> ohne Spannungsteiler
#define UBAT_DIVIDER		22000/8200			//Einzelwiderstände gemessen

#define LED_STATUS			GPIO_NUM_12			//Status - LED
#define LED_STATUS_REG		IO_MUX_GPIO12_REG	//MUX-Reg


#endif /* MAIN_INTERFACE_H_ */
