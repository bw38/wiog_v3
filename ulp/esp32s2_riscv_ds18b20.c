/* ULP-RISC-V example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This code runs on ULP-RISC-V  coprocessor
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "ulp_riscv/ulp_riscv.h"
#include "ulp_riscv/ulp_riscv_utils.h"
#include "ulp_riscv/ulp_riscv_gpio.h"

//#include "interface.h"


//#define GPIO_DEBUG		GPIO_NUM_13

//#define THRESHOLD 0.5 * 16

//Datenport wird durch init gesetzt
uint32_t owp = 0;

//Ergebnis an main
uint32_t temp_raw = 0;			//temp = (x & 0xFFFF) / 16.0
uint32_t crc_err = 0;			//0 => Ok / !=0 => Error

//Steuerung Messvorganng durch init
uint32_t max_force_report = 0;	//force wake nach x Messungen
uint32_t temp_threshold = 0;	//force wake bei delta temp x*16 (x=8 => +/- 0.5°C)

static uint32_t operation_delay_ms = 750;

static int32_t last_reported = INT32_MIN;
static uint8_t force_report = 0;


//Maxim-CRC8 Lookup-Table
static uint8_t lookuptable[256] = {
	0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};


static void ds18b20_write_bit(bool bit)
{
    ulp_riscv_gpio_output_level(owp, 0);
    if (bit) {
        /* Must pull high within 15 us, without delay this takes 5 us */
        ulp_riscv_gpio_output_level(owp, 1);
    }

    /* Write slot duration at least 60 us */
    ulp_riscv_delay_cycles(60 * ULP_RISCV_CYCLES_PER_US);
    ulp_riscv_gpio_output_level(owp, 1);
}

static bool ds18b20_read_bit(void)
{
    bool bit;

    /* Pull low minimum 1 us */
    ulp_riscv_gpio_output_level(owp, 0);
    ulp_riscv_gpio_output_level(owp, 1);

    /* Must sample within 15 us of the failing edge */
    ulp_riscv_delay_cycles(5 * ULP_RISCV_CYCLES_PER_US);
    bit = ulp_riscv_gpio_get_level(owp);

    /* Read slot duration at least 60 us */
    ulp_riscv_delay_cycles(55 * ULP_RISCV_CYCLES_PER_US);

    return bit;
}

static void ds18b20_write_byte(uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        ds18b20_write_bit((data >> i) & 0x1);
    }
}

static uint8_t ds18b20_read_byte(void)
{
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        data |=  ds18b20_read_bit() << i;
    }
    return data;
}

static bool ds18b20_reset_pulse(void)
{
    bool presence_pulse;
    /* min 480 us reset pulse + 480 us reply time is specified by datasheet */
    ulp_riscv_gpio_output_level(owp, 0);
    ulp_riscv_delay_cycles(480 * ULP_RISCV_CYCLES_PER_US);

    ulp_riscv_gpio_output_level(owp, 1);

    /* Wait for ds18b20 to pull low before sampling */
    ulp_riscv_delay_cycles(60 * ULP_RISCV_CYCLES_PER_US);
    presence_pulse = ulp_riscv_gpio_get_level(owp) == 0;

    ulp_riscv_delay_cycles(420 * ULP_RISCV_CYCLES_PER_US);

    return presence_pulse;
}


int main (void) {

	//Test-IO zu Beginn hochtasten


    // Setup GPIO used for 1wire
    ulp_riscv_gpio_init(owp);
    ulp_riscv_gpio_input_enable(owp);
    ulp_riscv_gpio_output_enable(owp);
    ulp_riscv_gpio_set_output_mode(owp, RTCIO_MODE_OUTPUT_OD);
    ulp_riscv_gpio_pullup(owp);
    ulp_riscv_gpio_pulldown_disable(owp);

/*
    ulp_riscv_gpio_init(GPIO_DEBUG);
    ulp_riscv_gpio_output_enable(GPIO_DEBUG);
    ulp_riscv_gpio_set_output_mode(GPIO_DEBUG, RTCIO_MODE_OUTPUT_OD);
    ulp_riscv_gpio_pullup_disable(GPIO_DEBUG);
    ulp_riscv_gpio_pulldown_disable(GPIO_DEBUG);
	ulp_riscv_gpio_output_level(GPIO_DEBUG, 1);
*/
    //Messung initialisieren
    ds18b20_reset_pulse();
    // Start conversion
    ds18b20_write_byte(0xCC);
    ds18b20_write_byte(0x44);
    //Operation delay
    ulp_riscv_delay_cycles(operation_delay_ms * 1000 * ULP_RISCV_CYCLES_PER_US);

    ds18b20_reset_pulse();

    uint8_t scratchpad[9];	//result-array
    // Read scratchpad
    ds18b20_write_byte(0xCC);
    ds18b20_write_byte(0xBE);

    uint8_t crc = 0;
    for (int i=0; i<8; i++){
    	scratchpad[i] = ds18b20_read_byte();
    	crc = lookuptable[crc ^ scratchpad[i]];
    }

    crc_err = 1;
    temp_raw = INT32_MIN;
    if (crc == ds18b20_read_byte()) {
    	temp_raw  = scratchpad[1] << 8;
    	temp_raw |= scratchpad[0];
    	crc_err = 0;
    }

    force_report++;
    // Wakes up the main CPU if ...
    if (((abs(temp_raw - last_reported) >= temp_threshold) && (crc_err == 0)) ||	(force_report >= max_force_report)) {
    	force_report = 0;
    	last_reported = temp_raw;

//    	ulp_riscv_delay_cycles(50*1000 * ULP_RISCV_CYCLES_PER_US);
//    	ulp_riscv_gpio_output_level(GPIO_DEBUG, 0);
        ulp_riscv_wakeup_main_processor();
    }


    /* ulp_riscv_shutdown() is called automatically when main exits,
       main will be executed again at the next timeout period,
       according to ulp_set_wakeup_period()
     */
    return 0;
}
