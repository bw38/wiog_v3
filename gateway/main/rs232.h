/*
 * rs232.h
 *
 *  Created on: 21.02.2021
 *      Author: joerg
 */

#ifndef MAIN_RS232_H_
#define MAIN_RS232_H_

void uart0_init();
void rx_uart_event_task(void *pvParameters);

//Prototypen
void send_uart_frame(const void *pl, uint8_t szup, char cFTyp);
void logL(char *text);
void logE(char *text);
void logW(char *text);
void logLV(char *text, int val);


#endif /* MAIN_RS232_H_ */
