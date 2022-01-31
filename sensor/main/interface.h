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
//#define UID40319	// ESP32 - TestPlug, div Sensoren
//#define UID09073	// ESP32 - LiPo-Basisboard, Stepup & HAT - BME280 / DS18B20
#define UID27963	// ESP32 - AM2302 - Rohrmelder or, Li-Bat Direktversorgung
//#define UID24931	// ESP32 - AM2302 - Rohrmelder gr, Li-Bat Direktversorgung
//#define UID45425c	// ESP32S2 - TestPlug, div Sensoren


// -----------------------------------------------------------



#ifdef UID40319	// -------- Testboard  ESP32-Plug ----------------------
#define RFLAG_UBAT			1
#define RFLAG_DS18B20_FSM	2
#define RFLAG_SHT31			4

#define	WAKEUP_SOURCE		WS_ULP				//wakeup_src_t WS_ULP | WS_MAIN;

//OneWirePort DS18B20
#define	USE_ULP_FSM
#define GPIO_DS18B20_OWP	4
#define RTC_IO_DS18B20_OWP	10

#define UBAT_ADC_CHN		ADC1_CHANNEL_7		//Messeingang	GPIO35
//#define UBAT_DIV_GND		GPIO_NUM_33			//Spannungsteiler während Messung nach unten ziehen
#define UBAT_DIV_GND		-1	 				// -1 -> ohne Spannungsteiler
#define UBAT_DIVIDER		227500/39900
#define VREF				1120

#define	USE_I2C_MASTER
#define	I2C_MASTER_NUM		I2C_NUM_0
#define I2C_MASTER_FREQ_HZ	100000	//Frequenz in Hz, max 1MHz
#define I2C_MASTER_SDA_IO 	GPIO_NUM_26
#define I2C_MASTER_SCL_IO 	GPIO_NUM_27


//Dummies f. Kompatibilität
//Status-LED
#define LED_STATUS			GPIO_NUM_14			//Status - LED
#define LED_STATUS_REG		IO_MUX_GPIO14_REG	//MUX-Reg

//StepUp-Regler-Steuerung	wg ds18b20.S
#define USE_STEPUP_CTRL
#define GPIO_STEPUP_CTRL	32
#define RTC_IO_STEPUP_CTRL	9

#endif	//--------------------------------------------------------

#ifdef UID09073	// ESP32 - LiPo-Basisboard, Stepup & HAT - BME280 / DS18B20
#undef DEBUG_X

#define RFLAG_UBAT			1
#define RFLAG_DS18B20_FSM	2
#define RFLAG_BME280		4

#define	WAKEUP_SOURCE		WS_ULP				//wakeup_src_t WS_ULP | WS_MAIN;

//OneWirePort DS18B20
#define	USE_ULP_FSM
#define GPIO_DS18B20_OWP	4
#define RTC_IO_DS18B20_OWP	10

#define UBAT_ADC_CHN		ADC1_CHANNEL_7		//Messeingang	GPIO35
//#define UBAT_DIV_GND		GPIO_NUM_33			//Spannungsteiler während Messung nach unten ziehen
#define UBAT_DIV_GND		-1	 				// -1 -> ohne Spannungsteiler
#define UBAT_DIVIDER		33000/8200
#define VREF				1095

#define TEMP_THRESHOLD		0.25				// 1/10°C -Schritte
#define MAX_FORCE_REPORT	0					// nach max x Messungen aufwecken

#define	USE_I2C_MASTER
#define	I2C_MASTER_NUM		I2C_NUM_0
#define I2C_MASTER_FREQ_HZ	100000	//Frequenz in Hz, max 1MHz
#define I2C_MASTER_SDA_IO 	GPIO_NUM_16
#define I2C_MASTER_SCL_IO 	GPIO_NUM_17

//StepUp-Regler-Steuerung
#define USE_STEPUP_CTRL
#define GPIO_STEPUP_CTRL	32
#define RTC_IO_STEPUP_CTRL	9

//Dummies f. Kompatibilität
//Status-LED
//#define LED_STATUS			GPIO_NUM_13			//Status - LED
//#define LED_STATUS_REG		IO_MUX_GPIO13_REG	//MUX-Reg



#endif	//--------------------------------------------------------


#if defined UID27963 || defined UID24931		// ESP32 - AM2302 - Rohrmelder or/gr, Li-Bat Direktversorgung
#undef DEBUG_X									//unterdrückt die meisten UART-Ausgaben

#define RFLAG_UBAT			1
#define RFLAG_AM2302_FSM	2

#define	WAKEUP_SOURCE		WS_ULP				//wakeup_src_t WS_ULP | WS_MAIN;

//OneWirePort Am2302
#define	USE_ULP_FSM
#define AM2302_GPIO_OWP		26
#define AM2302_RTC_IO_OWP	7

//#define TEMP_THRESHOLD		0.3					// 1/10°C -Schritte
//#define HUMI_THRESHOLD		2					// 1/10% - Schritte
//#define MAX_FORCE_REPORT	30					// nach max x Messungen aufwecken


#define UBAT_ADC_CHN		ADC1_CHANNEL_7		//Messeingang	GPIO35
#define UBAT_DIV_GND		32					//Spannungsteiler während Messung nach unten ziehen
//#define UBAT_DIV_GND		-1	 				// -1 -> ohne Spannungsteiler
#define UBAT_DIVIDER		22000/8200
#define VREF				1100				//Dummy - eFuse VRef

//Dummies f. Kompatibilität
//Status-LED
#define USE_STATUS_LED
#define LED_STATUS			12			//Status - LED
#define LED_STATUS_REG		IO_MUX_GPIO12_REG	//MUX-Reg

#define RTC_IO_STEPUP_CTRL	17			//GPIO27 Dummy
#endif	//--------------------------------------------------------


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
#include "gadget/ubat/ubat.h"

#define RFLAG_UBAT			1
#define RFLAG_SHT31_RISCV	2

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



/*
//verfügbare Sensortypen Sensortypen
//Flag-Nummern können je Gerät beliebig (2^n) vergeben werden
#define RFLAG_UBAT			1
#define RFLAG_DS18B20_FSM	2
#define RFLAG_DS18B20_RISCV 2
#define RFLAG_AM2302		4
#define RFLAG_BME280		8
#define RFLAG_BME280_RISCV	8
#define RFLAG_SHT3C			16
#define RFLAG_SHT31			32
#define RFLAG_SHT31_RISCV	64
*/

/*
//StepUp-Regler-Steuerung
#define USE_STEPUP_CTRL
#define GPIO_STEPUP_CTRL	32
#define RTC_IO_STEPUP_CTRL	9

#define USE_LED_STATUS
#define LED_STATUS			GPIO_NUM_14			//Status - LED
#define LED_STATUS_REG		IO_MUX_GPIO14_REG	//MUX-Reg


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

*/

#endif /* MAIN_INTERFACE_H_ */
