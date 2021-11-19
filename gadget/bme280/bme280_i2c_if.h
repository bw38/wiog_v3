/*
 * bme280_i2c_if.h
 *
 *  Created on: 26.04.2019
 *      Author: joerg
 */

#ifndef MAIN_BME280_I2C_IF_H_
#define MAIN_BME280_I2C_IF_H_

#include "driver/i2c.h"
#include "BME280_driver-master/bme280.h"
#include "BME280_driver-master/bme280_defs.h"

typedef struct {
	uint32_t humidity;
	int32_t  temperature;
	uint32_t pressure;
	int8_t	 status;
} bme280_result_t;

//externe Funktionen
extern esp_err_t bme280_i2c_init(i2c_port_t _i2c_mport, bool wu);
extern void bme280_i2c_start(QueueHandle_t hQ, uint32_t flag);
extern bme280_result_t bme280_i2c_get_result();

#endif /* MAIN_BME280_I2C_IF_H_ */
