/*
 * am2302.h
 *
 *  Created on: 03.02.2020
 *      Author: joerg
 */

#ifndef MAIN_AM2302_AM2302_H_
#define MAIN_AM2302_AM2302_H_

int32_t  am2302_temperature;
uint32_t am2302_humidity;
int8_t   am2302_crc_check;

extern void am2302_init();
extern void am2302_start(uint32_t flag);


#endif /* MAIN_AM2302_AM2302_H_ */
