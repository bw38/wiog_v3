/*
 * gw_main.h
 *
 *  Created on: 21.02.2021
 *      Author: joerg
 */

#ifndef MAIN_GW_MAIN_H_
#define MAIN_GW_MAIN_H_

#include "../../wiog_include/wiog_system.h"
#include "../../wiog_include/wiog_data.h"

//Prioritäten und Slots (synchron in allen Nodes (incl gw))
node_info_block_t nib;

void broadcast_nib();

#endif /* MAIN_GW_MAIN_H_ */
