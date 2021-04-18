/*
 * rs232.c
 *
 *  Created on: 20.02.2021
 *      Author: joerg
 *
 *  Kommunikation mit Raspberry Pi - GW
 *
 */

#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "string.h"
#include "../../wiog_include/wiog_system.h"
#include "../../wiog_include/wiog_data.h"

#include "rs232.h"
#include "gw_main.h"


//f. CRC-Summe
union int16_tt {
	struct {
		char l8;
		char h8;
	};
	uint16_t i16;
};

//Prototypen
uint16_t crc16(const uint8_t *data_p, uint16_t length);


// -----------------------------------------------------------------------------------
//UART
//Consolausgabe und UART-Frames teilen sich UART0

#define UART_NEXT_TXD_PIN 1
#define UART_NEXT_RXD_PIN 3

//static const int RX_BUF_SIZE = 1024;
static QueueHandle_t uart0_queue;

void uart0_init() {
    const uart_config_t uart_config = {
//    	.baud_rate = 115200,
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_NEXT_TXD_PIN, UART_NEXT_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, 2048, 1024, 20, &uart0_queue, 0);
}


//Payload in Frame packen, Frame-Typ kennzeichnen, CRC-Summe berechnen und über UART senden
void send_uart_frame(const void *pl, uint16_t szup, char cFTyp)
{
	char gwLoad[256] = "$JR*";	//Max
	gwLoad[4] = cFTyp;
	gwLoad[5] = (char) (szup & 0xFF);
	gwLoad[6] = (char) ((szup >> 8) & 0xFF);
	if (szup > 0) memcpy(gwLoad + 7, pl, szup);
	gwLoad[szup + 7] = cFTyp;
	union int16_tt crc;
	crc.i16 = crc16((uint8_t*)gwLoad, szup + 7);
	gwLoad[szup + 8] = crc.l8;
	gwLoad[szup + 9] = crc.h8;
	gwLoad[szup + 10] = '\n';
	uart_write_bytes(UART_NUM_0, gwLoad, szup + 11);//zum RPi senden via UART
}

//#define BUF_SIZE (1024)
//#define RD_BUF_SIZE (BUF_SIZE)

// UART Zeichenempfang
IRAM_ATTR void rx_uart_event_task(void *pvParameters)
{
    uint8_t data[2048];
    int fcUART = 0;
    char rx_char;
    char cFTyp = ' ';
    union int16_tt lenPL;
    uint16_t ixPL = 0;
    uart_payload_t pl; //payload zur Weiterverarbeitung
    uint16_t ixLine = 0;
    uint8_t line[2048];	//gesamte Zeile f. CRC-Berechnung
    union int16_tt crc;

    uart_event_t event;

    while (1)
    {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY))
        {
            if (event.type == UART_DATA)
            {
            	const int rxBytes = uart_read_bytes(UART_NUM_0, data, event.size, portMAX_DELAY);

            	//const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
            	if (rxBytes == 0)//Timeout
            	{
            		fcUART = 0;
            		ixLine = 0;
            	}
            	for (int i = 0; i < rxBytes; i++)
            	{
            		//FlowControl UART-Rx-Frame
            		// $ R J * [cFTyp][lenPL(L)][lenPL(H)][Payload][cFTyp][lCRC](hCRC][\n]
            		// 0 1 2 3    4       5         6         7       8      9    10   11
            		rx_char = data[i];
            		line[ixLine] = data[i];
            		ixLine++;
            		switch (fcUART)	{
        				case 0:
        					if (rx_char == '$') {
        						fcUART++;
        						line[0] = rx_char;
        						ixLine = 1;
        					}
        					break;
        				case 1:
        					if (rx_char == 'R') fcUART++;
        					else fcUART = 0;
        					break;
        				case 2:
        					if (rx_char == 'J') fcUART++;
        					else fcUART = 0;
        					break;
        				case 3:
        					if (rx_char == '*') fcUART++;
        					else fcUART = 0;
        					break;
        				case 4:
        					cFTyp = rx_char;
        					fcUART++;
        					break;
        				case 5: 					//Datenlänge L
        					lenPL.l8 = rx_char;
        					fcUART++;
        					break;
        				case 6:						//Datenlänge H
        					lenPL.h8 = rx_char;
        					ixPL = 0;
        					if (lenPL.i16 > 0) {
        						pl.data_len = lenPL.i16;
        						fcUART++;
        					}
        					else fcUART = 0;
        					break;
        				case 7:
        					pl.data[ixPL] = rx_char;
        					if (++ixPL >= lenPL.i16) fcUART++;
        					break;
        				case 8:
        					if (rx_char == cFTyp) fcUART++;
        					else fcUART = 0;
        					break;
        				case 9:
        					crc.l8 = rx_char;
        					fcUART++;
        					break;
        				case 10:
        					crc.h8 = rx_char;
        					fcUART++;
        					break;
        				case 11:
        					fcUART = 0;

        					if ((rx_char == '\n') && (crc.i16 == crc16(line, ixLine-3)))
        					{
        						if (cFTyp == 'n') { 	// Node-Info-Block
        							memcpy(&nib , pl.data, pl.data_len);
        							broadcast_nib(&nib);
        						} //NIB

        						else if (cFTyp == 'd') {	//Device-Info-Block
        							bzero(&dib, sizeof(dib));
        							memcpy(&dib , pl.data, pl.data_len);
        						}

        						else if (cFTyp == 'y') {	//Test Loop

        							logLV("Empfangen: ", pl.data_len);
        							send_uart_frame(pl.data, pl.data_len, 'Y');
        						}


        						else if (cFTyp == 'a') {	//Datenframe an Device senden
        							dev_uid_t uid = ((payload_t*)pl.data)->man.uid;
        							send_data_frame(pl.data, pl.data_len, uid);
        						} //Frametyp 'a'


        						else if (cFTyp == 'c') {	//Wifi-Channel des Gateway setzen
        							uint8_t ch = pl.data[0];
        							if ((ch > 0) && (ch < 14)) {
        								wifi_channel = ch;
        								esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE);
        								logLV("Working-Channel: ", wifi_channel);
        							}
        						} //Frametyp 'c'

        						else if (cFTyp == 'p') {	//Ankündigung eines Datenframes
        							dev_uid_t uid;
        							memcpy(&uid, pl.data, sizeof(dev_uid_t));
        							notice_payload(uid, 1);	//1 -> Daten für uid vorhanden
        						}

        						else if (cFTyp == 'o') {	//unregister Dataframes
        							dev_uid_t uid;
        							memcpy(&uid, pl.data, sizeof(dev_uid_t));
        							notice_payload(uid, 0);	//0 -> keine Daten für uid vorhanden
        						}





/*
        						if (cFTyp == 'b') //Steuerbefehl an Actor oder Repeater senden
        						{
        							LED_RT_ON;
        							pl.wifi_channel = get_wifi_channel(); //tatsächlichen Arbeitskanal
        							switch (pl.species) {
        								case SPECIES_ACTOR:
        									esp_now_send(mac_actor, pl.data, lenPL);
        									if (repeater_is_present > 0 )
        										esp_now_send(mac_repeater, pl.data, lenPL);
        									break;
        								case SPECIES_REPEATER:
        									esp_now_send(mac_repeater, pl.data, lenPL);
        									break;
        							}
        						} //Frametyp 'b'


        						if (cFTyp == 't')  //Watchdog-Timer-Kette
        						{
        							xTaskCreate(wd_uart_event_task, "wd_uart_event_task", 1024, NULL, 12, NULL);
        							wd_timeout_min = 3;	//SW-Reset nach 3min ohne WD-Feed
        						} //Frametyp 't'

*/

        					} else {
        						logE("Datenfehler (UART) RPi -> ESP");
        					}
        					break;
            		} //switch
            	} // if Event-type
            } //if Queue
        } //for
    } //while
    vTaskDelete(NULL);
}

#define POLY 0x8408
/*
 //                                      16   12   5
 // this is the CCITT CRC 16 polynomial X  + X  + X  + 1.
 // This works out to be 0x1021, but the way the algorithm works
 // lets us use 0x8408 (the reverse of the bit pattern).  The high
 // bit is always assumed to be set, thus we only use 16 bits to
 // represent the 17 bit value.
 */
//Kommunikation m. Raspi (Pascal)
uint16_t crc16(const uint8_t *data_p, uint16_t length)
{
	unsigned char i;
	unsigned int data;
	unsigned int crc = 0xffff;

	if (length == 0) return (~crc);

	do {
		for (i = 0, data = (unsigned int) 0xff & *data_p++; i < 8; i++, data >>= 1) {
			if ((crc & 0x0001) ^ (data & 0x0001)) crc = (crc >> 1) ^ POLY;
			else crc >>= 1;
		}
	} while (--length);

	crc = ~crc;
	data = crc;
	crc = (crc << 8) | (data >> 8 & 0xff);

	return (crc);
}

// -----------------------------------------------------------------------------------------

//---------------------------------------------------------------
//Message und Wert als Frame an Pi senden
void logL(char *text)
{
	send_uart_frame((uint8_t *)text, strlen(text), 'L');
}

void logLV(char *text, int val)
{
	char str[64];
	sprintf(str, "%s%d", text, val);
	logL(str);
}

void logE(char *text)
{
	send_uart_frame((uint8_t *)text, strlen(text), 'E');
}
void logW(char *text)
{
	send_uart_frame((uint8_t *)text, strlen(text), 'W');
}

// -------------------------------------------------------------

