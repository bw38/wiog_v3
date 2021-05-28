/*
 * ds18b20.h
 *
 *  Created on: 03.02.2020
 *      Author: joerg
 */

#ifndef MAIN_DS18B20_DS18B20_H_
#define MAIN_DS18B20_DS18B20_H_

int32_t ds18b20_temperature;

extern void ds18b20_init(gpio_num_t owp1);
extern void ds18b20_start(uint32_t flag);


#endif /* MAIN_DS18B20_DS18B20_H_ */
