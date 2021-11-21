#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

#include "interface.h"

#define _STEPUP		RTC_IO_STEPUP_CTRL		//Steurung StepUp-Regulator
#define _OWP		RTC_IO_DS18B20_OWP		//OneWirePort


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


//Data-Section, Constant ------------------------------------
.data

//Maxim-CRC8 Lookup-Table
lookuptable:
.long       0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,\
            157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,\
            35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,\
            190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,\
            70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,\
            219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,\
            101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,\
            248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,\
            140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,\
            17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,\
            175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,\
            50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,\
            202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,\
            87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,\
            233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,\
            116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53



//Data-Section Variable ------------------------------------

.bss

//kein Wakeup nach 1. ULP-Lauf
first_run:
	.long 0

//aktueller Messwert
	.global temp_raw
temp_raw:
	.long 0


//Schwellwerte f. wakeup, gesetzt in main
	.global temp_threshold
temp_threshold:
	.long 0

//max Messzyklen ohne wake, wird durch Host nach jedem Wake gesetzt
	.global ncycles_force_wake
ncycles_force_wake:
	.long 0

	.global cycles	//16bit-Zähler ulp_cycles
cycles:
	.long 0

//Fehlerzählung CRC, optionale Auswertung
	.global crc_err
crc_err:
	.long 0

//Datenport - Dummy Kompatibilität mit RISCV
	.global owp
owp:
	.long 0

//zuletzt gesendeter Wert
temp_last_val:
	.long 0

scratchpad:		// Return 9 Bytes of data after an 0xBE command.
	.long 0		// Byte 0: Temperature LSB
	.long 0		// Byte 1: Temperature MSB
	.long 0		// Byte 2: Th Register
	.long 0		// Byte 3: Tl Register
	.long 0 	// Byte 4: Configuraion Register
	.long 0		// Byte 5: Reserved 
	.long 0		// Byte 6: Reserved
	.long 0		// Byte 7: Reserved
	.long 0		// Byte 8: CRC value


//Code-Section -------------------------------------------------

	.text
	.global entry
entry:

	//Co-Programm bei 1.Aufruf sofort in Halt schicken (ohne Wake !)
	move R1, first_run	//first_run nach Initialisierung == 0
	ld   R0, R1, 0
	jumpr start, 1, GE	//ab 2. Aufruf
	move R0, 1
	st   R0, R1, 0
	jump exit_halt

start:
	SET_PIN_HIGH _STEPUP	//Regler hochtasten
	wait 64000		//8ms Einschwingzeit

	//Cycle-Counter hochzählen
	move R1, cycles
	ld	 R0, R1, 0
	add  R0, R0, 1
	st	 R0, R1, 0

	SET_MODE_INPUT _OWP

	//allgem Register
	//R3 - Return-Adresse aus Sub-Routines

	//12 Bit Resoluton nach Power up -> wird nicht geändert
	//Transaction Converting Command
	move R3, mr1
	jump master_reset

mr1:
	move R0, 0xCC			//Skip ROM-Command
	move R3, m0
	jump write_byte

m0:
	move R0, 0x44			//Convert Temperature
	move R3, m1
	jump write_byte

	//Converting Time
m1:
	move R2, 2000
cti1:
	sub R2, R2, 1
	jump cti_exit, eq		//Notbremse falls Sensor nicht antwortet

	wait 40000 				//ca. 5ms
	SET_MODE_OUTPUT _OWP
	SET_PIN_LOW _OWP			//Startimpuls
	wait(16)

	SET_MODE_INPUT _OWP		//Bus high, pullup
	READ_PIN _OWP			//ready: R0 == 0x1
	jumpr cti1, 1, lt		//warten bis Sensor geantwortet hat

cti_exit:
	//Transaction Data-Read
	move R3, mr2
	jump master_reset

mr2:
	move R0, 0xCC			//Skip ROM-Command
	move R3, m2
	jump write_byte

m2:
	move R0, 0xBE			//Command: Read ScratchPad
	move R3, m3
	jump write_byte

	//Lesen Sensor
m3:
	move R3, scratchpad		//Mem-Pointer

	stage_rst				//Byte-Counter (9 Bytes)

rd_byte_loop1:
	move R1, 0				//Byte-Cache
	move R2, 0				//Bit-Counter

rd_bit_loop1:
	SET_MODE_OUTPUT _OWP
	SET_PIN_LOW _OWP		//Startimpuls
	wait(16) 				// 2 us
	SET_MODE_INPUT _OWP		//Bus high, pullup
	wait(120) 				// 15 us
	READ_PIN _OWP			// Datenbit => R0[0]
	lsh R0, R0, R2			//Bit 7..0
	or R1, R1, R0
	wait (360)				//45µs
	add R2, R2, 1
	move R0, R2
	jumpr rd_bit_loop1, 8, lt //next Bit

	st R1, R3, 0			//store Byte in ScratchPad
	add R3, R3, 1
	stage_inc 1
	jumps rd_byte_loop1, 9, lt //next byte (while (R0=R2) < 9)

	//alle Register frei
	//CRC8 - Prüfung
	stage_rst
	move r0, 0	//crc init
	move r3, scratchpad
crc8loop:
	ld   r1, r3, 0		//Wert in Scratch-Tabelle
	// R0 = R0 xor R1 ==> R0 = (R0 or R1)-(R0 and R1)
	or   R2, R0, R1
	and  R0, R0, R1
	sub  R0, R2, R0

	add  r2, r0, lookuptable
	ld   r0, r2, 0

	add  r3, r3, 1
	stage_inc 1
	jumps crc8loop, 8, lt	//ohne crc-Byte

	ld   r1, r3, 0			//crc-byte aus scratch
	sub  r0, r0, r1			//crc-check  if (r0 == 0) set zf
	jump crc_ok, EQ			//jump if ok

	//CRC-Fehlerzahähler +1 (max 16bit)
	move r1, crc_err
	ld   r0, r1, 0
	add  r0, r0, 1
	st   r0, r1, 0
	jump exit

crc_ok:
	//load temperature from scratchpad[0..1]
	move R3, scratchpad
	ld   R0, R3, 0			//LSB
	add  R3, R3, 1
	ld   r2, R3, 0			//MSB
	lsh  r2, r2, 8
	or   r0, r0, r2
	move R1, temp_raw
	st   r0, R1, 0			//R0 = Raw-Temp

	//Messwertfenster: 0xFC90 .. 0x7D0 (-55 .. 125′C)
	//Messwertfenster in positiven Bereich verschieben => Vermeidung von Nulldurchgängen
	.set offset, 1000
	add  R0, R0, offset		//R0: aktueller Wert + offset
	move R1, temp_last_val
	ld   R1, R1, 0
	add  R1, R1, offset		//R1: last_val + offset, Vergeleichswert
	move R2, temp_threshold
	ld   R2, R2, 0			//R2: zulässiges Delta

	add  R3, R1, R2			//Obere Grenze überschritten ?
	sub  R3, R3, R0			//R3 = oberer Grenzwert - aktueller Wert -> OV ?
	jump exit_wake, OV

	sub  R3, R1, R2			//Untere Grenze unterschritten ?
	sub  R3, R0, R3			//R3 = aktueller Wert - unterer Grenzwert -> OV ?
	jump exit_wake, OV

    //Zwangsaufweckung nach n Messvorgängen
exit:
	move R1, ncycles_force_wake
	ld   R0, R1, 0
	and  R0, R0, 0xFF		//Wert auf 255 begrenzen
	jump exit_wake, EQ		//Wert ist bereits 0
	sub  R0, R0, 1			//cnt -= 1
	jump exit_wake, EQ		//Zwangspunkt erreicht
	st   R0, R1, 0			//Zyklenzähler zurückschreiben
	jump exit_halt

exit_wake:
	//gesendeten Wert für nächsten Vergleich speichern
	move R1, temp_raw
	ld   R0, R1, 0
	move R1, temp_last_val
	st	 R0, R1, 0

	wake

exit_halt:
	halt

// -----------------------------------------------------
// Subroutines

	//zu schreibendes Byte in R0
	//Return-Addr in R3
write_byte:
	stage_rst		//Bitzähler => 0
wrloop:
	SET_MODE_OUTPUT _OWP
	SET_PIN_LOW _OWP	//start of slot (min 1 / max 15µs)
	move R1, R0		    //R1[0] => Tx-Bit
	and  R1, R1, 0x1
	jump wrlow, eq	    //R1 = 0 ?
	wait (16)			//2 µs
	SET_MODE_INPUT _OWP	//Write 1, pullup-resistor
wrlow:					//Write 0
	wait (640)			//80µs
	SET_MODE_INPUT _OWP	//slot end
	wait (16)			//2µs
	rsh R0, R0, 1		//next Bit
	stage_inc 1
	jumps wrloop, 8, LT
	jump R3				//return


master_reset:
	//Master-Reset DS18B20
	SET_MODE_OUTPUT _OWP
	SET_PIN_LOW _OWP
	WAIT 4000				// 500µs
	SET_MODE_INPUT _OWP
	//Slave sendet presence pulse -> wird nicht ausgewertet
	wait(4000) 				//500µs
	jump R3



