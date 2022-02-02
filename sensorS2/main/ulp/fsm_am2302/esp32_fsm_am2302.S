#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

#include "interface.h"

#define _OWP	AM2302_RTC_IO_OWP	// RTC-OneWire-Port
#define _STEPUP	RTC_IO_STEPUP_CTRL	//Control StepUp-Regulator


//Port-Status "rtc_port" read => R0
.macro READ_PIN rtc_port
	READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + \rtc_port, 1)
.endm

//Port-Mode => Input (floating / high impedance)
.macro SET_MODE_INPUT rtc_port
	WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_ENABLE_W1TC_S + \rtc_port, 1, 1)
.endm

//Port-Mode => Output
.macro SET_MODE_OUTPUT rtc_port
	WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_ENABLE_W1TS_S + \rtc_port, 1, 1)
.endm

//Set Port => 1
.macro SET_PIN_HIGH rtc_port
	WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + \rtc_port, 1, 1)
.endm

//Set Port => 0
.macro SET_PIN_LOW rtc_port
	WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + \rtc_port, 1, 1)
.endm

// Define variables, which go into .bss section (zero-initialized data)
	.bss

//kein Wakeup nach 1. ULP-Lauf
first_run:
	.long 0

//Messergebnisse
	.global humidity
humidity:
	.long 0

	.global temperature
temperature:
	.long 0


	.global crc8_value
crc8_value:
	.long 0

	//Schwellwerte f. wakeup, gesetzt in main
	.global temp_threshold
temp_threshold:
	.long 0

	.global humi_threshold
humi_threshold:
	.long 0

	.global ncycles_force_wake	//max Messzyklen ohne wake, wird durch Host nach jedem Wake gesetzt
ncycles_force_wake:
	.long 0

	.global cycles	//16bit-Zähler ulp_cycles
cycles:
	.long 0

//zuletzt gesendete Werte
temp_last_val:
	.long 0

humi_last_val:
	.long 0

/*
Note: the temperature and humidity data read by the host from the AM2302 is always the last measured value, such as the
two measurement interval is very long, continuous read twice to the second value of real-time temperature and humidity values,
while two readtake minimum time interval be 2S.
*/
/*
ULP running at 8Mhz  => 0.125us per cpu cycle. (lt. manual)
measured on example-device: 65535 cycle in 7.2ms => 0.11µs per cycle
*/

// Code goes into .text section
	.text
	.global entry
entry:

// Register general usage:
// R0 : Result GPIO_read (bit 0)
// R1 : Return-addr for Subroutines
// R2 : Timeout in Sync-Functions
// R3 : holds Result for 16-/8-Bit-Reading
/*
	//Co-Programm bei 1.Aufruf sofort in Halt schicken (ohne Wake !)
	move R1, first_run	//first_run nach Initialisierung == 0
	ld   R0, R1, 0
	jumpr start, 1, GE	//ab 2. Aufruf
	move R0, 1
	st   R0, R1, 0
	jump exit_halt
*/
start:
	SET_PIN_HIGH _STEPUP	//StepUp-Regler hochtasten
	wait 64000				//8ms Einschwingzeit

	//Cycle-Counter hochzählen
	move R1, cycles
	ld	 R0, R1, 0
	add  R0, R0, 1
	st	 R0, R1, 0

	SET_MODE_INPUT _OWP

	//init Result
	move r3, 0
	move r1, humidity
	st r3, r1, 0
	move r1, temperature
	st r3, r1, 0
	move r1, crc8_value
	st r3, r1, 0
	stage_rst

	move r1, m0
	jump start_pulse	//start new acquisitition
// delay btw. 2 DHT-cycles
// manual says 2s
// testet > 500ms btw 2 singel cycles ok
// choose: 1000ms
m0:	//x_high
m0c: wait 0xFFFF		//ca. 7.2ms (8.9MHz) (testet Oszi) (wait max 16bit)
	stage_inc 1
	jumps m0c, 137, lt	//stage_cnt max 8 bit
	//x_low
	move r1, m0a
	jump start_pulse	//start data-reading

	//release-Time T(go) 20-200µs
m0a:wait(80)	//wait min half time (10µs)
	//Response procedre H-L, L->H, H->L ( =start of bit39)
	//check for first H->L
	move r1, m1	//return-addr after jump
	jump sync_HL
m1:	wait(480)	//check first L->H (low-time 75 - 85 µs

	move r1, m2
	jump sync_LH
m2: wait 480	//check for second H->L

	move r1, m3
	jump sync_HL
m3:	wait 240

	//read 16/16/8 bits -> r3
	stage_rst		//bit-counter
m4a: move r3, 0
m4b: move r1, m4c
	jump sync_LH	//sync L->H inside bit
m4c: wait 400		//50µs threshold t(H) <50µs => 0 | >50µs => 1
	READ_PIN _OWP	 //bit -> r0
	lsh r3, r3, 1	 //r3 = r3 << 1
	or r3, r3, r0	 //r3 = r3 | r1
	stage_inc 1		 //Bitzähler
	jumpr m5, 1, lt	 //0-bit: without delay
	wait 400		 //1-bit: add 50µs delay

m5:	jumps m6, 16, eq //humidity ready
	jumps m7, 32, eq //temperature ready
	jumps m8, 40, eq //crc ready

	jump m4b		 //next bit

m6:	move r1, humidity
	st r3, r1, 0
	jump m4a		//start next word (temperature)

m7:	move r1, temperature
	st r3, r1, 0
	jump m4a		//start next byte (crc)

m8:	move r1, crc8_value
	st r3, r1, 0
	//alle Werte gespeichert
	// tempetature = x°C * 10 // bit15==1 ? 0 - x°C *10
	// humidity = x% * 10

	//Zwangsaufweckung bei Temperaturänderung > +/- temp_threshold
	//bei Vorzeichenwechsel wird unabhängig vom Delta geweckt
	//	bspw 0x8001 -> 0x0001 = delta 0.2°C
	move R0, temperature
	ld	 R0, R0, 0			//R0:aktuell gemessene Temperatur
	move R1, temp_last_val
	ld   R1, R1, 0			//R1: zuletzt gemeldete Temperatur
	move R2, temp_threshold
	ld   R2, R2, 0			//R2: zulässiges Delta

	add  R3, R1, R2			//Obere Grenze überschritten ?
	sub  R3, R3, R0			//R3 = oberer Grenzwert - aktueller Wert -> OV ?
	jump exit_wake, OV

	sub  R3, R1, R2			//Untere Grenze unterschritten ?
	sub  R3, R0, R3			//R3 = aktueller Wert - unterer Grenzwert -> OV ?
	jump exit_wake, OV

	//Zwangsaufweckung bei Humidityänderung > +/- humi_threshold
	move R0, humidity
	ld	 R0, R0, 0			//R0:aktuell gemessene Luftfeuchtigkeit
	move R1, humi_last_val
	ld   R1, R1, 0			//R1: zuletzt gemeldete Luftfeutigkeit
	move R2, humi_threshold
	ld   R2, R2, 0			//R2: zulässiges Delta

	add  R3, R1, R2			//Obere Grenze überschritten ?
	sub  R3, R3, R0			//R3 = oberer Grenzwert - aktueller Wert -> OV ?
	jump exit_wake, OV

	sub  R3, R1, R2			//Untere Grenze unterschritten ?
	sub  R3, R0, R3			//R3 = aktueller Wert - unterer Grenzwert -> OV ?
	jump exit_wake, OV

    //Zwangsaufweckung nach n Messvorgängen
	move R1, ncycles_force_wake
	ld   R0, R1, 0
	and  R0, R0, 0xFF		//Wert auf 255 begrenzen
	jump exit_wake, EQ		//Wert ist bereits 0
	sub  R0, R0, 1			//cnt -= 1
	jump exit_wake, EQ		//Zwangspunkt erreicht
	st   R0, R1, 0			//Zyklenzähler zurückschreiben
	jump exit_halt

exit_wake:
	//gesendete Werte für nächsten Vergleich speichern
	move R1, temperature
	ld   R0, R1, 0
	move R1, temp_last_val
	st	 R0, R1, 0
	move R1, humidity
	ld   R0, R1, 0
	move R1, humi_last_val
	st	 R0, R1, 0

rdy_wakeup:                   // Read RTC_CNTL_RDY_FOR_WAKEUP bit
	READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
	AND r0, r0, 1
	JUMP rdy_wakeup, eq    // Retry until the bit is set
	wake	//Wake up the SoC and stop ULP program

exit_halt:
    halt

exit_error:	//timeout
	move r1, crc8_value //fake crc
	ld r3, r1, 0
	add r3, r3, 1		//leads to crc-check error in main
	jump m8


// Sub-routines ---------------------------------------

start_pulse:				//Start - Command to DTH22
	SET_MODE_OUTPUT _OWP	//out-mode
	SET_PIN_LOW _OWP		//Start-Signal T(be) (low: 800 - 2000µs)
	wait 8000				//1000µs (=> 1000 / 0.125)
	SET_PIN_HIGH _OWP
	SET_MODE_INPUT _OWP		//input-mode
	jump r1

//Level ist high, waits for low-level
sync_HL:
	move r2, 1000
x1:	READ_PIN _OWP	//Data => r0
	sub r2, r2, 1	//r2 -= 1
	jump exit_error, eq	//timeout
	jumpr x1, 1, ge	//loop while 1
	jump r1			//return-addr

//Level ist low, waits for high-level
sync_LH:
	move r2, 1000
x2: READ_PIN _OWP
	sub r2, r2, 1
	jump exit_error, eq
	jumpr x2, 1, lt
	jump r1
