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

//Ausgabe-Bits
#define OUT_BIT_A1			GPIO_NUM_25
#define INIT_OUT_BIT_A1		gpio_set_direction(OUT_BIT_A1, GPIO_MODE_INPUT_OUTPUT)
#define OUT_BIT_A1_ON		gpio_set_level(OUT_BIT_A1, 1)
#define OUT_BIT_A1_OFF		gpio_set_level(OUT_BIT_A1, 0)
#define OUT_BIT_A1_SET(X)	gpio_set_level(OUT_BIT_A1, X)

#define OUT_BIT_A2			GPIO_NUM_26
#define INIT_OUT_BIT_A2		gpio_set_direction(OUT_BIT_A2, GPIO_MODE_INPUT_OUTPUT)
#define OUT_BIT_A2_ON		gpio_set_level(OUT_BIT_A2, 1)
#define OUT_BIT_A2_OFF		gpio_set_level(OUT_BIT_A2, 0)
#define OUT_BIT_A2_SET(X)	gpio_set_level(OUT_BIT_A2, X)

#define OUT_BIT_B1			GPIO_NUM_32
#define INIT_OUT_BIT_B1		gpio_set_direction(OUT_BIT_B1, GPIO_MODE_INPUT_OUTPUT)
#define OUT_BIT_B1_ON		gpio_set_level(OUT_BIT_B1, 1)
#define OUT_BIT_B1_OFF		gpio_set_level(OUT_BIT_B1, 0)
#define OUT_BIT_B1_SET(X)	gpio_set_level(OUT_BIT_B1, X)

#define OUT_BIT_B2			GPIO_NUM_33
#define INIT_OUT_BIT_B2		gpio_set_direction(OUT_BIT_B2, GPIO_MODE_INPUT_OUTPUT)
#define OUT_BIT_B2_ON		gpio_set_level(OUT_BIT_B2, 1)
#define OUT_BIT_B2_OFF		gpio_set_level(OUT_BIT_B2, 0)
#define OUT_BIT_B2_SET(X)	gpio_set_level(OUT_BIT_B2, X)

#define NS_ON	0x0A	//einziges zulässiges Bitmuster zum Einschalten der Niederspannung
#define NS_OFF	0x05	//Niederspannung aus



// Meldekontakte
#define IN_BIT_X			GPIO_NUM_4
#define IN_BIT_Y			GPIO_NUM_5
#define GPIO_INPUT_PIN_SEL	((1ULL<<IN_BIT_X) | (1ULL<<IN_BIT_Y));


#endif /* MAIN_INTERFACE_H_ */
