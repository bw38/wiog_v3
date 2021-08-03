/*
 * sonoff.c
 *
 *  Created on: 02.08.2021
 *      Author: joerg
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "wiog_system.h"


#define GPIO_RELAIS		GPIO_NUM_12		//LED bl fest mit Relaisfunktion verbunden
#define GPIO_LED_GN		GPIO_NUM_13
#define GPIO_BUTTON		GPIO_NUM_0

static xQueueHandle gpio_evt_queue = NULL;

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

static void gpio_button_task(void *arg)
{
    uint32_t io_num;

    while (true) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}


void init_device() {
	//Ausgänge
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL<<GPIO_RELAIS) | (1ULL<<GPIO_LED_GN));
    io_conf.pull_down_en = 0;	//disable pull-down mode
    io_conf.pull_up_en = 0;		//disable pull-up mode
    gpio_config(&io_conf);		//configure GPIO

    //Button
    io_conf.intr_type = GPIO_INTR_POSEDGE;		//interrupt of rising edge
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<GPIO_BUTTON);
    io_conf.pull_down_en = 0;	//disable pull-down mode
    io_conf.pull_up_en = 0;		//disable pull-up mode  - extern R18
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_button_task, "botton", 2048, NULL, 10, NULL);

}
