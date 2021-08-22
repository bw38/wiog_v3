/*
 * sonoff.c
 *
 *  Created on: 02.08.2021
 *      Author: joerg
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "wiog_system.h"
#include "wiog_data.h"
#include "sonoff.h"

#define GPIO_RELAIS		GPIO_NUM_12		//LED bl fest mit Relaisfunktion verbunden
#define GPIO_LED_GN		GPIO_NUM_13
#define GPIO_BUTTON		GPIO_NUM_0

#define BM_NS_ON		0x01	//einziges zulässiges Bitmuster zum Einschalten der Niederspannung
#define BM_NS_OFF		0x00	//Niederspannung aus

#define SET_NS_ON		 gpio_set_level(GPIO_RELAIS, BM_NS_ON)
#define SET_NS_OFF		 gpio_set_level(GPIO_RELAIS, BM_NS_OFF)
#define GET_NS_STATE 	 gpio_get_level(GPIO_RELAIS)
#define GET_BUTTON_STATE (gpio_get_level(GPIO_BUTTON) ^ 1)	//Button down => 1


static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle ns_control_queue = NULL;

uint32_t ns_uptime_sek = 0;
uint32_t gpio_button = GPIO_BUTTON;
bool no_delay = false;


//Grüne LED nachleuchten lassen
void led_gn_flash_task(void *arg)
{
	gpio_set_level(GPIO_LED_GN, 0);
	vTaskDelay(200*MS);
	gpio_set_level(GPIO_LED_GN, 1);
	vTaskDelete(NULL);
}

void led_gn_flash(void)
{
	xTaskCreate(led_gn_flash_task, "led_gn_flash", 512, NULL, 1, NULL);
}

//-----------------------------------------------------------------------------------

//Status sofort senden
void main_send_immediately() {
	uint32_t flag = 0;
	xQueueSend(measure_response_queue, &flag, portMAX_DELAY);
}

//Status verzögert senden
IRAM_ATTR static void main_send_delayed_task(void *pvParameter) {
	if (!no_delay) vTaskDelay(300*MS);
	no_delay = false;
	uint32_t flag = 0;
	xQueueSend(measure_response_queue, &flag, portMAX_DELAY);
	vTaskDelete(NULL);
}

void main_send_delayed() {
	xTaskCreate(main_send_delayed_task, "main_send_task", 1024, NULL, 1, NULL);
}

//-----------------------------------------------------------------------------------
// zentrale Steurung des Schaltzustandes
void ns_control_task(void *arg) {
	uint32_t tsek;
	ns_control_queue = xQueueCreate(5, sizeof(uint32_t));

	while(true) {
		int x = GET_NS_STATE;
		if(xQueueReceive(ns_control_queue, &tsek, 1000 * MS) == pdPASS) {
			ns_uptime_sek = tsek;
			if (tsek == 0) SET_NS_OFF;
			else SET_NS_ON;
		} else {
			if (ns_uptime_sek > 0) {
				ns_uptime_sek--;
				if (ns_uptime_sek == 0) SET_NS_OFF;
			}
		}
		//bei Zustandsänderung main informieren
		if (x != GET_NS_STATE) main_send_delayed();
	}
}

//---------------------------------------------------------------------------------------------

static void button_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    gpio_isr_handler_remove(gpio_num);
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


static const char *TAG = "button";
static void gpio_button_task(void *arg)
{
    uint32_t gpio_num;

    while (true) {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "GPIO[%d] intr, val: %d\n", gpio_num, gpio_get_level(gpio_num));
            vTaskDelay(50 * MS);
            if (GET_BUTTON_STATE == 1) {	//Taster noch gedrückt ?
            	no_delay = true;	//Statusmeldung unverzögert senden
            	device_set_ns_state(device_get_ns_state() ^ 1);
            }
            gpio_isr_handler_add(GPIO_BUTTON, button_isr_handler, (void *) gpio_button);
        }
    }
}

//----------------------------------------------------------------------------------------------

//1 - NS On, alle anderen Werte NS Off
void device_set_ns_state(int x) {
	uint32_t state = 0;
	if (x == 1) state = UINT32_MAX; //ns_uptime 136Jahre
	xQueueSend(ns_control_queue, &state, portMAX_DELAY);
}

//Timer in Sek setzen / 0 - NS Off
void device_set_ns_timer(int sek) {
	xQueueSend(ns_control_queue, &sek, portMAX_DELAY);
}

int32_t device_get_ns_state() {
	return GET_NS_STATE;
}

int32_t device_get_button_state() {
	return GET_BUTTON_STATE;
}

void device_set_status_led(int x) {
	if (x == 1)
		gpio_set_level(GPIO_LED_GN, 1);
	else
		gpio_set_level(GPIO_LED_GN, 0);
}

void device_init() {
	//Ausgänge
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL<<GPIO_RELAIS) | (1ULL<<GPIO_LED_GN));
    io_conf.pull_down_en = 0;	//disable pull-down mode
    io_conf.pull_up_en = 0;		//disable pull-up mode
    gpio_config(&io_conf);		//configure GPIO

    //Button
    io_conf.intr_type = GPIO_INTR_NEGEDGE;		//interrupt edge
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<GPIO_BUTTON);
    io_conf.pull_down_en = 0;	//disable pull-down mode
    io_conf.pull_up_en = 0;		//disable pull-up mode  - extern R18
    gpio_config(&io_conf);

    //queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_button_task, "botton", 2048, NULL, 10, NULL);
    //Steuerung Schaltzustand NS
    xTaskCreate(ns_control_task, "ns_control", 2048, NULL, 5, NULL);

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_BUTTON, button_isr_handler, (void *) gpio_button);


}
