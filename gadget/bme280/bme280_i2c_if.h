/*
 * bme280_i2c_if.h
 *
 *  Created on: 26.04.2019
 *      Author: joerg
 */

#ifndef MAIN_BME280_I2C_IF_H_
#define MAIN_BME280_I2C_IF_H_

#include "driver/i2c.h"
#include "bme280.h"

uint32_t bme280_humidity;
int32_t  bme280_temperature;
uint32_t bme280_pressure;

RTC_DATA_ATTR static int8_t rtc_bme280_init_result;

//externe Funktionen
extern esp_err_t bme280_i2c_master_init(i2c_port_t i2c_master_port, int sda_io_num, int scl_io_num, bool wu);
extern void bme280_i2c_start(uint32_t flag);

#endif /* MAIN_BME280_I2C_IF_H_ */
