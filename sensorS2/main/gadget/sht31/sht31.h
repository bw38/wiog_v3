/*
 * shtc3_i2c_if.h
 *
 *  Created on: 30.09.2021
 *      Author: joerg
 */

#ifndef MAIN_SHT31_I2C_IF_H_
#define MAIN_SHT31_I2C_IF_H_

#include "driver/i2c.h"


typedef struct {
	uint32_t humidity;
	int32_t  temperature;
	int8_t	 status;
} sht31_result_t;

//externe Funktionen
extern esp_err_t sht31_i2c_init(i2c_port_t _i2c_mport, bool wu);
extern void sht31_i2c_start(QueueHandle_t hQ, uint32_t flag);
extern sht31_result_t sht31_i2c_get_result();


#endif /* MAIN_SHT31_I2C_IF_H_ */
