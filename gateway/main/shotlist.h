/*
 * shotlist.h
 *
 *  Created on: 07.12.2021
 *      Author: joerg
 */

#ifndef MAIN_SHOTLIST_H_
#define MAIN_SHOTLIST_H_

#include "freertos/FreeRTOS.h"
#include "wiog_include/wiog_system.h"

typedef struct {
	int ix, top;
	dev_uid_t *list;
} shotlist_t;

void shotlist_init(shotlist_t *psl, int len);
void shotlist_free(shotlist_t *psl);
bool shotlist_add(shotlist_t *psl, int32_t val);
bool shotlist_get(shotlist_t *psl, int32_t val);

#endif /* MAIN_SHOTLIST_H_ */
