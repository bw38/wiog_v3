/*
 * ns_socket.c
 *
 *  Created on: 30.01.2021
 *      Author: joerg
 *
 *   Steckdose V1.1 / Mai 2020
 */


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "ns_socket.h"
#include "interface.h"


static int64_t point_in_time = 0;	//Zeitpunkt Abschaltung NS, Vergleich Systemzeit (µs)

bool timer_task_is_running = false;

static xQueueHandle gpio_evt_queue = NULL;

//Prototypen
//uint32_t get_out_bitmask();
void ns_time_point(void *pvParameters);
bool set_out_bitmask(uint8_t ns);
void indicator_task(void* arg);

// ----------------------------------------------------------------------------------------------


// Schnittstellen main **************************************************************************************

void device_init() {
    //Ausgabe-Pins -----------------------------------------------------------------
    INIT_OUT_BIT_A1;
    INIT_OUT_BIT_A2;
    set_out_bitmask(NS_OFF);

    INIT_OUT_BIT_GN; OUT_BIT_GN_OFF;
    INIT_OUT_BIT_BL; OUT_BIT_BL_OFF;

    //Eingabe Interrupt -------------------------------------------------------------
    //Indicatoren
    xTaskCreate(indicator_task, "indicator", 2048, NULL, 10, NULL); //start gpio task

    xTaskCreate(ns_time_point, "ns_state", 2048, NULL, 15, NULL); //Überwachung Schaltzustand NS
}


//Status sofort senden
void main_send_immediately() {
	uint32_t flag = 0;
	xQueueSend(measure_response_queue, &flag, portMAX_DELAY);
}

//Status verzögert senden
IRAM_ATTR static void main_send_delayed_task(void *pvParameter) {
	vTaskDelay(500*MS);
	uint32_t flag = 0;
	xQueueSend(measure_response_queue, &flag, portMAX_DELAY);
	vTaskDelete(NULL);
}

// Bitmaske für Steuerung
void device_set_control(uint32_t bm){
	uint8_t ns = NS_OFF;
	if (bm == 1) ns = NS_ON;
	set_out_bitmask(ns);
	xTaskCreate(main_send_delayed_task, "main_send_task", 1024, NULL, 1, NULL);
}

void device_set_timer(uint32_t sek) {
	point_in_time = esp_timer_get_time() + sek*1000*1000;
	device_set_control(1);
}

/*
//Gerätestatus zurückmelden
void device_get_status()
{
	//Schaltzustand zurückliefern
	union data_entry_t dem = {
			.type = SW_ONOFF,
			.status = 0,
			.value = get_out_bitmask() == NS_ON};
	main_add_entry(dem);

	//Bitmaske des Tasters negiert zurückliefern
	union data_entry_t dei = {
			.type = SW_INDICATOR,
			.status = 0,
			.value = gpio_get_level(IN_BIT_X) ^ 1};	//Low-Activ
	main_add_entry(dei);

	//Restlaufzeit in Sek
	union data_entry_t det = {
			.type = SW_ON_SEK,
			.status = 0,
			.value = (point_in_time - esp_timer_get_time()) / 1e6};
	main_add_entry(det);
}
*/

//Bitmaske des Tasters negiert zurückliefern
uint32_t device_get_in_bitmask() {
	return gpio_get_level(IN_BIT_X) ^ 1;	//Low-Activ
}


// *************************************************************************************************************

//lesen der aktuell gesetzen Ausgangs-Maskierung
uint32_t device_get_out_bitmask() {
	uint32_t mask = 0;
	mask |= gpio_get_level(OUT_BIT_A2); mask <<= 1;
	mask |= gpio_get_level(OUT_BIT_A1);
	return mask;
}

//Ausgabesteuerung entspr Bitmuster
bool set_out_bitmask(uint8_t ns) {
	uint32_t nsx = device_get_out_bitmask();	//Ausgangszustand merken

	if (ns != NS_ON) ns = NS_OFF;
	//Ausgabebits setzen
	OUT_BIT_A1_SET(ns & 0x01);
	OUT_BIT_A2_SET(ns & 0x02);

	//Status-LED GN setzen
	uint32_t nsy = device_get_out_bitmask();
	if (nsy == NS_ON)
		OUT_BIT_GN_ON;
	else
		OUT_BIT_GN_OFF;
	//nach Zustandsänderung Daten an GW senden
	return nsy == nsx;
}

// -----------------------------------------------------------------------------

//Überwachung der Laufzeit
void ns_time_point(void *pvParameters) {
	while(true)	{
		uint32_t mask = device_get_out_bitmask();
		if (mask != NS_OFF) {
			//NS Off bei Zeitüberschreitung
			if ((point_in_time > 0) &&(esp_timer_get_time() > point_in_time)) {
				set_out_bitmask(NS_OFF);
				main_send_immediately();
			}
		}
		vTaskDelay(1000*MS);	//1x je Sekunde
	}
}

// ----------------------------------------------------------------------------
//Meldekontakte

//ISR-Routine des Tasters
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    gpio_set_intr_type(gpio_num, GPIO_INTR_DISABLE);	//erneute Freigabe erst nach Bearbeitung
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


//Tastenüberwachung
void indicator_task(void* arg)
{
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));//create a queue to handle gpio event from isr

	gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;	//interrupt on both edges
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;	//bit mask input
    io_conf.mode = GPIO_MODE_INPUT;				//set as input mode
    io_conf.pull_up_en = 1;						//enable pull-up mode
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(IN_BIT_X, gpio_isr_handler, (void*) IN_BIT_X);

    uint32_t io_num;
    while (true)  {
    	io_num = 0;
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
        	vTaskDelay(50*MS);	//Störimpulsunterdrückung
        	if (device_get_in_bitmask() == 1  ) {
        		uint32_t ns = NS_OFF;
        		if (device_get_out_bitmask() == NS_OFF) {
        			ns = NS_ON;
        			point_in_time = 0;	// => infinity
        		}
        		else {
        			ns = NS_OFF;
        			point_in_time = 0;
        		}
        		set_out_bitmask(ns);
        		main_send_immediately();
				int btup = 4;
				while (btup > 0) {
					vTaskDelay(250*MS);	//Entprellzeit
					if (gpio_get_level(IN_BIT_X)==0) btup = 4;
					btup--;
				}
        	}
			gpio_set_intr_type(IN_BIT_X, GPIO_INTR_NEGEDGE);
        }
    }
}
