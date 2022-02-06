/*
 * interface.h
 *
 * Geräte spezifikation der Sensoren
 */

#ifndef MAIN_INTERFACE_H_
#define MAIN_INTERFACE_H_

/*
 * Sensorauswahl ----------------------------------------
 * 1. UID auswählen (nur eine Auswahl zulässig, alle anderen auskommentieren
 * 2. ESP-Typ wählen / Terget in Eclipse oder "idf.py set-target esp32XX
 * 3. Komponenten in main/CMakefiles.txt auswählen / aktivieren
 * 4. sdkconfig anpassen (ULP, RISCV, reservierter ULP-Speicher ....)
 * 5. build, testen
 * 6. #undef Debug_X -> build
 */


//Geräteauswahl (nur eine UID zulässig -----------------------
#define UID45425c	// ESP32S2 - TestPlug, div Sensoren


// -----------------------------------------------------------




#ifdef UID45425a				// Testboard ESP32S2 Plug - RISC-V DS18B20 / SHT31 Main ----------------
#define RFLAG_UBAT			1
#define RFLAG_DS18B20_RISCV	2
#define RFLAG_SHT31			4

#define	WAKEUP_SOURCE		WS_ULP				//wakeup_src_t WS_ULP | WS_MAIN;

//OneWirePort DS18B20
#define	USE_ULP_RISCV
#define GPIO_DS18B20_OWP	10
#define RTC_IO_DS18B20_OWP	10
#define TEMP_THRESHOLD		0.25	//main bei Temperaturänderung wecken
#define MAX_FORCE_REPORT	5		//nach x Messung main sätestens wecken

#define UBAT_ADC_CHN		ADC1_CHANNEL_3		//GPIO 4 Messeingang
//#define UBAT_DIV_GND		GPIO_NUM_xx			//Spannungsteiler während Messung nach unten ziehen
#define UBAT_DIV_GND		-1	 				// -1 -> ohne Spannungsteiler
#define UBAT_DIVIDER		227500/39900
#define VREF				1100

#define	USE_I2C_MASTER
#define	I2C_MASTER_NUM		I2C_NUM_0
#define I2C_MASTER_FREQ_HZ	100000	//Frequenz in Hz, max 1MHz
#define I2C_MASTER_SDA_IO 	GPIO_NUM_8
#define I2C_MASTER_SCL_IO 	GPIO_NUM_9


//Dummies f. Kompatibilität
//Status-LED
#define LED_STATUS			GPIO_NUM_14			//Status - LED
#define LED_STATUS_REG		IO_MUX_GPIO14_REG	//MUX-Reg

//StepUp-Regler-Steuerung	wg ds18b20.S
#define USE_STEPUP_CTRL
#define GPIO_STEPUP_CTRL	2
#define RTC_IO_STEPUP_CTRL	2

#endif	//--------------------------------------------------------

#ifdef UID45425b			// Testboard ESP32S2 Plug - RISC-V BME280 Main ----------------

#define RFLAG_UBAT			1
#define RFLAG_BME280_RISCV	2

#define	WAKEUP_SOURCE		WS_ULP				//wakeup_src_t WS_ULP | WS_MAIN;

#define UBAT_ADC_CHN		ADC1_CHANNEL_3		//GPIO 4 Messeingang
//#define UBAT_DIV_GND		GPIO_NUM_xx			//Spannungsteiler während Messung nach unten ziehen
#define UBAT_DIV_GND		-1	 				// -1 -> ohne Spannungsteiler
#define UBAT_DIVIDER		227500/39900
#define VREF				1100				//Dummy - 2Point

#define	USE_ULP_RISCV
#define I2C_MASTER_SDA_IO 	8
#define I2C_MASTER_SCL_IO 	9

#endif	//--------------------------------------------------------


#ifdef UID45425c			// Testboard ESP32S2 Plug - RISC-V SHT31 --------------------------------
#define RFLAG_UBAT			1
#define RFLAG_SHT31_RISCV	2

#undef DEBUG_Xx
#define	WAKEUP_SOURCE		WS_ULP				//wakeup_src_t WS_ULP | WS_MAIN;

#define UBAT_ADC_CHN		ADC1_CHANNEL_1		//GPIO 2 Messeingang
//#define UBAT_DIV_GND		GPIO_NUM_xx			//Spannungsteiler während Messung nach unten ziehen
#define UBAT_DIV_GND		-1	 				// -1 -> ohne Spannungsteiler
#define UBAT_DIVIDER		22000/8200
#define VREF				1100				//Dummy - 2Point

#define	USE_ULP_RISCV
#define I2C_MASTER_SDA_IO 	GPIO_NUM_8
#define I2C_MASTER_SCL_IO 	GPIO_NUM_9

//StepUp-Regler-Steuerung
#define USE_STEPUP_CTRL
#define GPIO_STEPUP_CTRL	GPIO_NUM_20
#define RTC_IO_STEPUP_CTRL	GPIO_NUM_20

#endif	//-- UID45425c ------------------------------------------------------



#endif /* MAIN_INTERFACE_H_ */
