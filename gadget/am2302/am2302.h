/*
 * am2302.h
 *
 *  Created on: 03.02.2020
 *      Author: joerg
 */

#ifndef MAIN_AM2302_AM2302_H_
#define MAIN_AM2302_AM2302_H_

typedef struct {
	int32_t  temperature;
	uint32_t humidity;
	int8_t   crc_check;
} am2302_result_t;



/*
extern void am2302_init();
extern void am2302_start(uint32_t flag);
extern am2302_result_t am2302_get_result();
*/
#endif /* MAIN_AM2302_AM2302_H_ */
