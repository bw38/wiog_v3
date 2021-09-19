/*
 * pumpe.c
 *
 *  Created on: 30.01.2021
 *      Author: joerg
 *
 *   Steuerung Wasserpumpe Garten / Repeater Garage
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

#include "interface.h"
#include "pumpe.h"


typedef enum {
	TIME_TINY_MS,
	TIME_SMALL_MS,
	TIME_LONG_MS
} ns_up_time_t;


#define DEFAULT_TINY_TIME_MS	1 * 60 * 1000
#define DEFAULT_SMALL_TIME_MS	7 * 60 * 1000
#define DEFAULT_LONG_TIME_MS	20 * 60 * 1000

#define MIN_NS_ON_TIME_MS		10 * 01 * 1000	//Min zulässige Einschaltzeit
#define MAX_NS_ON_TIME_MS		25 * 60 * 1000	//Max zulässige Einschaltzeit, danach Zwangsreset

static int64_t point_in_time;	//Zeitpunkt Pumpenabschaltung (NS)

bool timer_task_is_running = false;

static xQueueHandle gpio_evt_queue = NULL;

//Prototypen
void monitoring_ns_state(void *pvParameters);
void nvs_set_nsup_time(ns_up_time_t upt, uint32_t time_ms);
uint32_t nvs_get_nsup_time(ns_up_time_t upt);
void set_out_bitmask(uint8_t ns);
void indicator_task(void* arg);

// ----------------------------------------------------------------------------------------------


// Schnittstellen main **************************************************************************************

void device_init() {
    //Ausgabe-Pins -----------------------------------------------------------------
    INIT_OUT_BIT_A1; OUT_BIT_A1_ON;
    INIT_OUT_BIT_A2; OUT_BIT_A2_OFF;
    INIT_OUT_BIT_B1; OUT_BIT_B1_ON;
    INIT_OUT_BIT_B2; OUT_BIT_B2_OFF;

    //Eingabe Interrupt -------------------------------------------------------------
    //Indicatoren
    xTaskCreate(indicator_task, "indicator", 2048, NULL, 10, NULL); //start gpio task

    xTaskCreate(monitoring_ns_state, "ns_state", 2048, NULL, 15, NULL); //Überwachung Schaltzustand NS
}

//Status sofort senden
void main_send_immediately() {
	uint32_t flag = 0;
	xQueueSend(measure_response_queue, &flag, portMAX_DELAY);
}

// Bitmaske für Pumpensteuerung
void device_set_control(uint32_t bm){
	uint8_t ns = NS_OFF;
	if (bm ==1) ns = NS_ON;
	set_out_bitmask(ns);
	main_send_immediately();
}



//Bitmaske der IN-Bits negiert zurückliefern
uint32_t device_get_in_bitmask() {
	uint32_t mask = 0;
	mask |= gpio_get_level(IN_BIT_Y)^1; mask <<= 1;
	mask |= gpio_get_level(IN_BIT_X)^1;
	return mask;
}

//lesen der aktuell gesetzen Ausgangs-Maskierung
uint32_t device_get_out_bitmask() {
	uint32_t mask = 0;
	mask |= gpio_get_level(OUT_BIT_B2); mask <<= 1;
	mask |= gpio_get_level(OUT_BIT_B1); mask <<= 1;
	mask |= gpio_get_level(OUT_BIT_A2); mask <<= 1;
	mask |= gpio_get_level(OUT_BIT_A1);
	return mask;
}

//Ausgabesteuerung entspr Bitmuster
void set_out_bitmask(uint8_t ns)
{
	OUT_BIT_A1_SET(ns & 0x01);
	OUT_BIT_A2_SET(ns & 0x02);
	OUT_BIT_B1_SET(ns & 0x04);
	OUT_BIT_B2_SET(ns & 0x08);
}

// -----------------------------------------------------------------------------

//Überwachung der Maximallaufzeit
void monitoring_ns_state(void *pvParameters)
{
	int64_t ts = esp_timer_get_time();
	while(true)
	{
		uint32_t mask = device_get_out_bitmask();
		if (mask != NS_OFF) {
			//Zwangsreset bei Zeitüberschreitung
			if ((esp_timer_get_time() - ts) > ((int64_t)MAX_NS_ON_TIME_MS * 1000) )
				esp_restart();
		} else
			ts = esp_timer_get_time();

		vTaskDelay(1000*MS);	//1x je Sekunde
	}
}

// ----------------------------------------------------------------------------
//Meldekontakte

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    gpio_set_intr_type(gpio_num, GPIO_INTR_DISABLE);	//erneute Freigabe erst nach Bearbeitung
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

//Timer-Task, während Laufzeit ist NS230V (Pumpe) eingeschaltet
void timer_ns_task(void* arg)
{
	timer_task_is_running = true;
	bool terminated = false;
	uint8_t cnt_off = 0;
	while (!terminated)
	{
		if (esp_timer_get_time() < point_in_time)
		{
			set_out_bitmask(NS_ON);
			vTaskDelay(500*MS);
		}
		else terminated = true;

		//Timer beenden bei dauerhaft gedr. Außentaster
		if (gpio_get_level(IN_BIT_X) == 0) cnt_off++; else cnt_off = 0;
		if(cnt_off > 4) terminated = true;

		//Kontrolle unsinniger Werte
		if (point_in_time > (esp_timer_get_time() + (int64_t)MAX_NS_ON_TIME_MS * 1000))
				terminated = true;
	}
	//NS230V ausschalten
	set_out_bitmask(NS_OFF);
	main_send_immediately();	//Status an Gateway
	timer_task_is_running = false;
	vTaskDelete(NULL);
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
    gpio_isr_handler_add(IN_BIT_Y, gpio_isr_handler, (void*) IN_BIT_Y);

    uint32_t io_num;
    TaskHandle_t h_timer_ns = NULL;
    int64_t dbl_click_us = 0;
    while (true)
    {
    	io_num = 0;
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
        	vTaskDelay(50*MS);	//Entprellzeit

        	gpio_set_intr_type(IN_BIT_X, GPIO_INTR_NEGEDGE);
        	gpio_set_intr_type(IN_BIT_Y, GPIO_INTR_NEGEDGE);

        	if ((io_num == IN_BIT_X) && (gpio_get_level(IN_BIT_X) == 0))	//Außentaster
        	{
        		ns_up_time_t nsup;
        		if (esp_timer_get_time() < (dbl_click_us + 1000*1000)) //2x Click innerhalb 1s
        			nsup = TIME_SMALL_MS;	//def 7min
        		else												//erster Click oder Einzelclick
        			nsup = TIME_TINY_MS;	//def 1min

        		dbl_click_us = esp_timer_get_time();
        		point_in_time = esp_timer_get_time() + (int64_t)nvs_get_nsup_time(nsup) * 1000;

            	if (!timer_task_is_running)
            		xTaskCreate(timer_ns_task, "timer_ns", 2048, NULL, 5, &h_timer_ns);

            	main_send_immediately();	//Status an Gateway
        	}
        	else if ((io_num == IN_BIT_Y) && (gpio_get_level(IN_BIT_Y) == 0)) //Innentaster (Longterm)
        	{
        		//Langzeittimer nur starten, wenn Timer derzeit nicht aktiv
        		if (!timer_task_is_running)
        		{
        			point_in_time = esp_timer_get_time() + (int64_t)nvs_get_nsup_time(TIME_LONG_MS) * 1000;
        			xTaskCreate(timer_ns_task, "timer_ns", 2048, NULL, 5, &h_timer_ns);
        			main_send_immediately();	//Status an Gateway
        		}
        		else
        			point_in_time = 0;	//sonst Task schließen
        	}
        }
    }
}


//NS-Einschaltzeiten in ms in NVS ablegen
//Bereichskontrolle
void nvs_set_nsup_time(ns_up_time_t upt, uint32_t time_ms)
{
	if ((time_ms >= MIN_NS_ON_TIME_MS) && (time_ms <= MAX_NS_ON_TIME_MS ))
	{
		nvs_handle hnvs;
		if (nvs_open("storage", NVS_READWRITE, &hnvs) == ESP_OK)
		{
			switch (upt)
			{
				case TIME_TINY_MS:
					nvs_set_u32(hnvs, "time_tiny_ms", time_ms);
					break;
				case TIME_SMALL_MS:
					nvs_set_u32(hnvs, "time_small_ms", time_ms);
					break;
				case TIME_LONG_MS:
					nvs_set_u32(hnvs, "time_long_ms", time_ms);
					break;
			}
		}
	}
}


uint32_t nvs_get_nsup_time(ns_up_time_t upt)
{
	nvs_handle hnvs;
	uint32_t res = 0;
	uint32_t defres = 0;
	if (nvs_open("storage", NVS_READONLY, &hnvs) == ESP_OK)
	{
		switch(upt)
		{
		case TIME_TINY_MS:
			if (nvs_get_u32(hnvs, "time_tiny_ms", &res) !=  ESP_OK) res = 0;
			defres = DEFAULT_TINY_TIME_MS;
			break;
		case TIME_SMALL_MS:
			if (nvs_get_u32(hnvs, "time_small_ms", &res) !=  ESP_OK) res = 0;
			defres = DEFAULT_SMALL_TIME_MS;
			break;
		case TIME_LONG_MS:
			if (nvs_get_u32(hnvs, "time_long_ms", &res) !=  ESP_OK) res = 0;
			defres = DEFAULT_LONG_TIME_MS;
			break;
		}
	}
	if ((res < MIN_NS_ON_TIME_MS) || (res > MAX_NS_ON_TIME_MS)) res = defres;

	return res;
}
