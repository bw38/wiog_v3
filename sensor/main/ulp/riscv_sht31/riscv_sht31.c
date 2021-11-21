/* ULP-RISC-V example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This code runs on ULP-RISC-V  coprocessor
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

//relative pathe are not found in eclipse - IDF ok
#include "ulp_riscv/ulp_riscv.h"
#include "ulp_riscv/ulp_riscv_utils.h"
#include "ulp_riscv/ulp_riscv_gpio.h"

uint32_t sht31_sda = 0;	//individual ports
uint32_t sht31_scl = 0;

#define SHT31_I2C_ADDR_L	0x44	//Device Addr
#define SHT31_I2C_ADDR_H	0x45

#define SHT31_SW_RESET 		0x30A2
#define SHT31_READ_STATUS	0xF32D
#define SHT31_CLEAR_STATUS	0x3041

//Start-Modes - Single Shot - ClockStretching disabled
#define SHT31_MODE_LOW		0x2416
#define SHT31_MODE_MED		0x240B
#define SHT31_MODE_HIGH		0x2400

#define SHT31_HEATER_EN		0x306D
#define SHT31_HEATER_DIS	0x3066

#define CLK			20 	//ca. 25kHz Takt (gemessen)
#define T25			ulp_riscv_delay_cycles(CLK / 4)
#define	T1MS		ulp_riscv_delay_cycles(1000 * ULP_RISCV_CYCLES_PER_US);
#define	T3MS		ulp_riscv_delay_cycles(3000 * ULP_RISCV_CYCLES_PER_US);

//control SCL /SDA
//Grundstellung High -> Input (ext. PullUp) / aktiv Low => Input & Output (fix low)
#define SCL_L		ulp_riscv_gpio_output_enable(sht31_scl)
#define SCL_H		ulp_riscv_gpio_output_disable(sht31_scl)
#define X_SCL		ulp_riscv_gpio_get_level(sht31_scl)
#define SDA_L		ulp_riscv_gpio_output_enable(sht31_sda)
#define SDA_H		ulp_riscv_gpio_output_disable(sht31_sda)
#define X_SDA		ulp_riscv_gpio_get_level(sht31_sda)

//interne var ------------------------------------------------------------------
static uint32_t mcycle;

static uint32_t mtemp = 0;;
static uint32_t mhumi = 0;

//global var -------------------------------------------------------------------
//return result
uint32_t sht31_humi, sht31_temp;	// Res*100
uint32_t sht31_status_reg;			// Bit 15..0, s. Datenblatt
uint32_t sht31_err;					// 0 = Ok
uint32_t sht31_cycles = 0;
//control mainprocess
uint32_t sht31_set_heater = 0;		// Heizung 0-off / 1-on
uint32_t sht31_set_mode = 1;		// 0-Low, 1-Medium, 2-High
//wakeup conditions
uint32_t set_sht31_thres_temp = 50;	// +/- 0.5°C
uint32_t set_sht31_thres_humi = 200;	// +/- 2%
uint32_t set_sht31_force_wake = 0;	// max alle x Messungen -> Meldung an MainProc

//prototypes ------------------------------------------------------------------
static void init_gpio();
static uint8_t checkCRC(uint16_t packet, uint8_t cs);

//------------------------------------------------------------------------------

static void tx_start_bit() {
	SDA_H; T25; SCL_H; T25; SDA_L; T25;	// -> SDA_L
}

static void tx_stop_bit() {	//SCL_H
	T25; SCL_L; SDA_L; T25; SCL_H; T25; SDA_H;
}

static void tx_1_bit() {	//SCL H
	T25; SCL_L; T25; SDA_H; T25; SCL_H; T25;
}

static void tx_0_bit() {	//SCL H
	T25; SCL_L; T25; SDA_L; T25; SCL_H; T25;	// -> SDA_L
}

static uint8_t rx_bit() {
	uint8_t x;
	T25;  SCL_L; SDA_H; T25; x = X_SDA; SCL_H;  T25;
	return x;
}

static void tx_ack_bit() {
	T25; SCL_L; T25; SDA_L; T25; SCL_H; T25;		// ->SDA_L
}

static void tx_nack_bit() {
	T25; SCL_L; T25; SDA_H; T25; SCL_H; T25;
}

static void tx_byte (uint8_t x) {
	for (int i = 0; i < 8; i++) {
		if ((x & 0x80) == 0) tx_0_bit();
		else tx_1_bit();
		x = x << 1;
	}
}

//16-Bit cmd to SHT3x
//return 0 => Ok
static uint8_t tx_cmd (uint16_t cmd) {
    tx_start_bit();
    tx_byte((SHT31_I2C_ADDR_L << 1) & 0xFE);
    uint8_t x = rx_bit();
    tx_byte((uint8_t)(cmd >> 8));
    x |= rx_bit();
    tx_byte((uint8_t)(cmd));
    x |= rx_bit();
    tx_stop_bit();
    return x;
}

//read data sequentially
//acnowledge n-1 ACK / last byte send NACK
static void rx_bytes(uint8_t *data, uint8_t n) {
	uint8_t x;
	uint8_t b;
	for (int j = 0; j < n; j++) {
		for (int i = 0; i < 8; i++) {
			x <<= 1;
			b = rx_bit();
			x |= b;
		}
		data[j] = x;
		if (j < (n-1)) tx_ack_bit();
	}
	tx_nack_bit();
}


int main (void) {
	sht31_cycles++;
	init_gpio();

    uint16_t raw_temp, raw_humi;

    uint8_t rx_data[6];
    //first run or error before
    if (sht31_err != 0) {
    	sht31_err = tx_cmd(SHT31_SW_RESET);
    	T3MS;
    }

    //read status register
    if (sht31_err == 0) {
    	sht31_err = tx_cmd(SHT31_READ_STATUS);
    	//3 Bytes lesen
    	tx_start_bit();	// ?: tx_cmd closed with stopbit
    	tx_byte((SHT31_I2C_ADDR_L << 1) | 0x01);
    	sht31_err |= rx_bit();
    	rx_bytes(rx_data, 3);
    	tx_stop_bit();
    	//Result
    	uint16_t sr = rx_data[0] * 256 + rx_data[1];
    	sht31_err |= checkCRC(sr, rx_data[2]);
    	if (sht31_err == 0) {
    		sht31_status_reg = sr;
    	} else {
    		sht31_status_reg = 0xFFFFFFFF;
    	}
    }

    if (sht31_err == 0) {
    	if (((sht31_status_reg & 0x2000) == 0) && (sht31_set_heater == 1)) tx_cmd(SHT31_HEATER_EN);
    	if (((sht31_status_reg & 0x2000) != 0) && (sht31_set_heater == 0)) tx_cmd(SHT31_HEATER_DIS);
    }

    //activate measurement
    if (sht31_err == 0) {
    	uint16_t m;
    	switch (sht31_set_mode) {
    		case 1:	 m = SHT31_MODE_MED; break;
    		case 2:  m = SHT31_MODE_HIGH; break;
    		default: m = SHT31_MODE_LOW;
    	}
    	sht31_err = tx_cmd(m);
    }

    //wait for measure end
    if (sht31_err == 0) {
    	uint8_t x = 1;
    	uint8_t y = 0;
    	//sht3x writes NACK (x==1) as log as no data available
    	while (x == 1) {
    		T1MS;
    		if (y++ > 50) {
    			sht31_err = 1;
    			break;		//emergency-exit
    		}
    		tx_start_bit();
    		tx_byte((SHT31_I2C_ADDR_L << 1) | 0x01);
    		x = rx_bit();	//ACK ? NACK
    		if (x==1) tx_stop_bit();
    	}
    }

    //read raw data - Temp -> Humi
    if (sht31_err == 0) {
    	rx_bytes(rx_data, 6);
    	tx_stop_bit();
    	raw_temp = rx_data[0] * 256 + rx_data[1];
    	raw_humi = rx_data[3] * 256 + rx_data[4];
    	//CRC-Summe test
    	sht31_err = checkCRC(raw_temp, rx_data[2]) | checkCRC(raw_humi, rx_data[5]);
    }

    //Data for main-process
    //data correction, s. doc
    if (sht31_err == 0) {
    	sht31_temp = (17500 * raw_temp / 65535) - 4500;
    	sht31_humi = 10000 * raw_humi / 65535;
    } else {
    	sht31_temp = 0;	// err != 0
    	sht31_humi = 0;
    }

    //wakeup conditions

    if ((++mcycle >= set_sht31_force_wake) ||				//max cycles seit letzter Meldung
    	(abs(mtemp - sht31_temp) >= set_sht31_thres_temp) ||	//nach Temperaturänderung
		(abs(mhumi - sht31_humi) >= set_sht31_thres_humi)){		//nach Luftfeuchteänderung
    	mtemp = sht31_temp;
    	mhumi = sht31_humi;
    	mcycle = 0;
    	ulp_riscv_wakeup_main_processor();
    }



    return 0;	//Shutdown until next wakeup
}

// Setup GPIO for bit by bit
static void init_gpio() {
    ulp_riscv_gpio_init(sht31_sda);
    ulp_riscv_gpio_input_enable(sht31_sda);
    ulp_riscv_gpio_set_output_mode(sht31_sda, RTCIO_MODE_OUTPUT_OD);
    ulp_riscv_gpio_pullup_disable(sht31_sda);
    ulp_riscv_gpio_pulldown_disable(sht31_sda);
    ulp_riscv_gpio_output_level(sht31_sda, 0);

    ulp_riscv_gpio_init(sht31_scl);
    ulp_riscv_gpio_input_enable(sht31_scl);
    ulp_riscv_gpio_set_output_mode(sht31_scl, RTCIO_MODE_OUTPUT_OD);
    ulp_riscv_gpio_pullup_disable(sht31_scl);
    ulp_riscv_gpio_pulldown_disable(sht31_scl);
    ulp_riscv_gpio_output_level(sht31_scl, 0);
}


//ok => return 0
static uint8_t checkCRC(uint16_t packet, uint8_t cs)
{
	uint8_t upper = packet >> 8;
	uint8_t lower = packet & 0x00FF;
	uint8_t data[2] = {upper, lower};
	uint8_t crc = 0xFF;
	uint8_t poly = 0x31;

	for (uint8_t indi = 0; indi < 2; indi++) {
		crc ^= data[indi];
		for (uint8_t indj = 0; indj < 8; indj++) {
			if (crc & 0x80)	{
				crc = (uint8_t)((crc << 1) ^ poly);
			} else {
				crc <<= 1;
			}
		}
	}
	return (cs ^ crc);
}

