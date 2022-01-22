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
#include "shotlist.h"


//f. CRC-Summe
union uint16_tt {
	struct {
		char l8;
		char h8;
	};
	uint16_t i16;
};


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
	union uint16_tt crc;
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
    union uint16_tt lenPL;
    uint16_t ixPL = 0;
    uart_payload_t pl; //payload zur Weiterverarbeitung
    uint16_t ixLine = 0;
    uint8_t line[2048];	//gesamte Zeile f. CRC-Berechnung
    union uint16_tt crc;

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

        						else if (cFTyp == 'r') {	//ESP-GW Reset
        							esp_restart();
        						}

        						else if (cFTyp == 'x') {	//Device resetten
        							dev_uid_t uid = ((payload_t*)pl.data)->man.uid;
        							shotlist_add(&sl_uid_reset, uid);

        						}
        						else if (cFTyp == 'a') {	//Datenframe an Device senden
        							dev_uid_t uid = ((payload_t*)pl.data)->man.uid;
        							if (get_device_info(uid)->species == SENSOR) {
        								logLV("Error: Data to Sensor", uid);
        							} else {
        								//Actor: Datensatz sofort senden
        								send_data_frame((payload_t*)&pl.data, pl.data_len, uid);
        							}
        						} //Frametyp 'a'


        						else if (cFTyp == 'c') {	//Wifi-Channel des Gateway setzen
        							uint8_t ch = pl.data[0];
        							if ((ch > 0) && (ch < 14)) {
        								wifi_channel = ch;
        								esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE);
        								logLV("Working-Channel: ", wifi_channel);
        							}
        						} //Frametyp 'c'
/*
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
	send_uart_frame((uint8_t *)text, strlen(text)+1, 'E');
}
void logW(char *text)
{
	send_uart_frame((uint8_t *)text, strlen(text)+1, 'W');
}

// -------------------------------------------------------------

