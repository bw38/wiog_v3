/*
 * 	ubat.c
	Modul - Batteriespannungsmessung
 */

#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"


#include "ubat.h"

#include "../../wiog_include/wiog_data.h"


#define DEBUG_VREF_NO	//Caliebrierungsverfahren bestimmen und URef an GPIO25 messen

//Batteriemessung
#define	UBAT_SAMPLES	100				//Messzyklen zur Rauschminderung
#define MIN_VREF		1000			//lt Datenblatt
#define MAX_VREF		1200			//dto
#define DEFAULT_VREF	1100				//falls nicht in efuse und nicht selbst in NVS eingetragen
#define ADC_ATTEN 		ADC_ATTEN_DB_0	//Abschwächer

#define DEBUG_VREF_NO

esp_adc_cal_characteristics_t *padc_chars;
SemaphoreHandle_t	UBat_Semaphore = NULL;

uint32_t samples;
adc1_channel_t adc_chn;	//Messeingang
gpio_num_t gpio_gnd;	//Masse Spannungsteiler

//extern
int	Ubat_ADC_in_mV;		//Messergebnis an ADC-Eingang in mV

esp_adc_cal_value_t vref_cal_type;
uint32_t rflag;

//Prototypen
uint32_t nvs_get_vref();

// Batteriespannung ---------------------------------------------------------------------------------

//static int get_UBat(int samples)
static void get_ubat_task(void * pvParameters)
{
	//Batteriespannung messen
	uint32_t uadc = 0;
	for (int u=0; u < samples; u++) //100 Samples ca. 4.3ms
	{
		uadc += adc1_get_raw((adc1_channel_t)adc_chn);
	}

	//Spannungsteiler hochohmig ausschalten
	if (gpio_gnd < GPIO_NUM_MAX) gpio_set_level(gpio_gnd, 1);

	Ubat_ADC_mV = esp_adc_cal_raw_to_voltage(uadc / samples, padc_chars); //Spannung mV am Messeingang

	xQueueSend(measure_response_queue, &rflag, portMAX_DELAY);	//Messung freigeben
    vTaskDelete(NULL);
}

void ubat_init(adc1_channel_t ch, gpio_num_t in_gnd, uint32_t spl) {
	adc_chn = ch;
	samples = spl;

	if (in_gnd >= 0) gpio_gnd = in_gnd;
	else gpio_gnd = GPIO_NUM_MAX;
	//Spannungsteiler f. Batteriespannungsmessung einschalten (extern ca. 22KOhm/8,2kOhm)
	//bei 100nF gegen Masse -> Spannung nach ca. 8ms stabil
	if (gpio_gnd < GPIO_NUM_MAX) {
		gpio_set_direction(gpio_gnd, GPIO_MODE_OUTPUT);
		gpio_set_pull_mode(gpio_gnd, GPIO_FLOATING);
		gpio_set_level(gpio_gnd, 0);
	}
}

// Port initialisieren und Task starten
void ubat_start(uint32_t flag) {

	rflag = flag;

	uint32_t dev_vref = nvs_get_vref();
	if ((dev_vref < 950) || (dev_vref > 1250)) dev_vref = DEFAULT_VREF;

	adc1_config_width(ADC_WIDTH_BIT_12); 			//Sample-Breite
	adc1_config_channel_atten(adc_chn, ADC_ATTEN);	//Abschwächer

	//Characterize ADC
	padc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	vref_cal_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, dev_vref, padc_chars);

	//Task starten
	xTaskCreate(get_ubat_task, "ubat", 1024, NULL, 2, NULL);


#ifdef DEBUG_VREF
    //Characterize ADC at particular atten
    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("eFuse Vref\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Two Point\n");
    } else {
        printf("Default\n");
//        esp_err_t status = adc2_vref_to_gpio(GPIO_NUM_26);
        //Verzögerung zum linearen messen
//        vTaskDelay(15000 / portTICK_PERIOD_MS);
    }
#endif

 //   return dev_vref;
}


// --------------------------------------------------------------------

uint32_t nvs_get_vref()	// < 1ms
{
  	uint32_t res = 0;
   	nvs_handle hnvs;
   	if (nvs_open("storage", NVS_READONLY, &hnvs) == ESP_OK)
   	{
   		if (nvs_get_u32(hnvs, "vref", &res) !=  ESP_OK) res = 0;
   		nvs_close(hnvs);
   	}
   	return res;
}


void ubat_set_vref(uint32_t vref) // 3 ms
{
   	nvs_handle hnvs;
   	if (nvs_open("storage", NVS_READWRITE, &hnvs) == ESP_OK)
   	{
   		nvs_set_u32(hnvs, "vref", vref);
   		nvs_close(hnvs);
   	}
}








