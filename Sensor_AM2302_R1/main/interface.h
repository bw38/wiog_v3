/*
 * interface.h
 *
 *  Created on: 28.05.2021
 *      Author: joerg
 */

#ifndef MAIN_INTERFACE_H_
#define MAIN_INTERFACE_H_

//StepUp-Regler-Steuerung
//#define GPIO_STEPUP_CTRL	32
//#define RTC_IO_STEPUP_CTRL	9

//OneWirePort DS18B20
//#define GPIO_DS18B20_OWP	27
//#define RTC_IO_DS18B20_OWP	17

#define AM2302_GPIO_OWP		26
#define AM2302_RTC_IO_OWP	7

#define UBAT_ADC_CHN		ADC1_CHANNEL_7		//Messeingang	GPIO35
#define UBAT_DIV_GND		GPIO_NUM_32			//Spannungsteiler während Messung nach unten ziehen
//#define UBAT_DIV_GND		-1	 				// -1 -> ohne Spannungsteiler
#define UBAT_DIVIDER		22000/8200			//Einzelwiderstände gemessen

//#define BME280_I2C_MASTER_NUM		I2C_NUM_0	//0 od. 1
//#define BME280_I2C_MASTER_SCL_IO	25			//Clock --> SCL     (Pin 3)
//#define BME280_I2C_MASTER_SDA_IO	26			//Data <--> SDA/SDI (Pin 4)

#endif /* MAIN_INTERFACE_H_ */
