/*
 * interface.h
 *
 *  Created on: 28.05.2021
 *      Author: joerg
 */

#ifndef MAIN_INTERFACE_H_
#define MAIN_INTERFACE_H_


#define UIDxxxxx

//StepUp-Regler-Steuerung
#define GPIO_STEPUP_CTRL	32
#define RTC_IO_STEPUP_CTRL	9

#define LED_STATUS			GPIO_NUM_14			//Status - LED
#define LED_STATUS_REG		IO_MUX_GPIO14_REG	//MUX-Reg

//OneWirePort DS18B20
#define GPIO_DS18B20_OWP	4
#define RTC_IO_DS18B20_OWP	10

//#define AM2302_GPIO_OWP		14
//#define AM2302_RTC_IO_OWP	16

#define UBAT_ADC_CHN		ADC1_CHANNEL_7		//Messeingang	GPIO35
//#define UBAT_DIV_GND		GPIO_NUM_33			//Spannungsteiler während Messung nach unten ziehen
#define UBAT_DIV_GND		-1	 				// -1 -> ohne Spannungsteiler
#define UBAT_DIVIDER		22000/8200			//Einzelwiderstände gemessen

//Chipabhängige Referenzspannung
#ifdef UIDxxxxx
	#define VREF			1100
#endif

#define BME280_I2C_MASTER_NUM		I2C_NUM_0	//0 od. 1
#define BME280_I2C_MASTER_SCL_IO	17			//Clock --> SCL     (Pin 3)
#define BME280_I2C_MASTER_SDA_IO	16			//Data <--> SDA/SDI (Pin 4)

#endif /* MAIN_INTERFACE_H_ */
