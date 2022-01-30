/*
 * interface.h
 *
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

#define OUT_BIT_GN			GPIO_NUM_18
#define INIT_OUT_BIT_GN		gpio_set_direction(OUT_BIT_GN, GPIO_MODE_INPUT_OUTPUT)
#define OUT_BIT_GN_ON		gpio_set_level(OUT_BIT_GN, 0)	//Low-Aktiv
#define OUT_BIT_GN_OFF		gpio_set_level(OUT_BIT_GN, 1)

#define OUT_BIT_BL			GPIO_NUM_19
#define INIT_OUT_BIT_BL		gpio_set_direction(OUT_BIT_BL, GPIO_MODE_INPUT_OUTPUT)
#define OUT_BIT_BL_ON		gpio_set_level(OUT_BIT_BL, 0)
#define OUT_BIT_BL_OFF		gpio_set_level(OUT_BIT_BL, 1)

#define NS_ON	0x02	//einziges zulässiges Bitmuster zum Einschalten der Niederspannung
#define NS_OFF	0x01	//Niederspannung aus

#define LED_STATUS_OFF		OUT_BIT_BL_OFF
#define LED_STATUS_ON		OUT_BIT_BL_ON

// Meldekontakte
#define IN_BIT_X			GPIO_NUM_17
#define GPIO_INPUT_PIN_SEL	(1ULL<<IN_BIT_X);
#define IN_BIT_X_VAL		gpio_get_level(IN_BIT_X);



#endif /* MAIN_INTERFACE_H_ */
