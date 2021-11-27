/*
 * interface.h
 *
 *  Created on: 07.06.2021
 *      Author: joerg
 *
 *      Steuerung bder Regentonnenpumpe
 *      Repeater Garage
 *
 *      Statusmeldung Actor -> GW
 *      	Datatype dt_bitmsk
 *      	IX=0 -> Out-bitmask / Triac-Steurung
 *      		0x0A - NS_ON
 *      		0x05 - NS_OFF
 *      		alle anderen Werte ungültig -> NS_OFF
 *
 *      	ix=1 -> In-Bitmask / Status Taster - 1=aktiv
 *      		Bit0 - Außentaster
 *      		Bit1 - Innentaster - Langzeit
 *
 */

#ifndef MAIN_INTERFACE_H_
#define MAIN_INTERFACE_H_




#endif /* MAIN_INTERFACE_H_ */
