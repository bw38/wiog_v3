/*
 * pumpe.h
 *
 *  Created on: 30.01.2021
 *      Author: joerg
 *
 *  Steuerung Wasserpumpe Garten / Repeater Garage
 *
 */

#ifndef MAIN_PUMPE_PUMPE_H_
#define MAIN_PUMPE_PUMPE_H_

#define MS 1/portTICK_PERIOD_MS	//1 Tick je 10ms


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




extern void pumpctrl_init(xQueueHandle hQR);
extern void pumpctrl_set_control(uint32_t x, uint32_t tsek);
extern uint32_t pumpctrl_get_in_bitmask();
extern uint32_t pumpctrl_get_out_bitmask();


#endif /* MAIN_PUMPE_PUMPE_H_ */
