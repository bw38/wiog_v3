/*
 *  Internet og Garden
 *  ==================
 *  gemeinsame Funktionen ab Version 3.0
 *
 *  C-File wird über das Dateisystem in den main-folder der Projekte verlinkt (ln)
 *  PREFIX "ln_" -> Eintrag in seafile-ignore.txt
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "stdint.h"
#include "string.h"
#include "nvs_flash.h"
#include "esp_err.h"

#include "mbedtls/aes.h"
#include "esp32/rom/aes.h"
#include "esp32/rom/crc.h"

#include "wiog_system.h"


esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}


// --------------------------------------------------------------------------------

uint64_t debug_timers[8];
void tstart(uint8_t ix ) {
	debug_timers[ix] = esp_timer_get_time();
}

int tstop(uint8_t ix) {
	int res = esp_timer_get_time() - debug_timers[ix];
	printf("Time: %dµs\n", res);
	return res;
}


void hexdump(uint8_t *data, int len) {
	printf("\n");
	for( int i = 0; i < len; i++ )
	{
		printf( "%02x%c", data[i], ((i&0xf)!=0xf)?' ':'\n' );
	}
	printf( "\n" );
}

void hexdumpex(uint8_t *data, int len) {
	printf("\n");
	for( int i = 0; i < len; i++ )
	{
		printf( "%02x[%c]%c", data[i], ((data[i]>31)&(data[i]!=0x7f))?data[i]:' ', ((i&0xf)!=0xf)?' ':'\n' );
	}
	printf( "\n" );
}


//resultuerende Blocklänge ist ein aufgerundetes Vielfaches der Schlüssellänge
int get_blocksize(int data_len) {
	//Blocklänge als Vielfaches von KEYLEN
	uint8_t key[] = {AES_KEY};	//CBC-AES-Key
	uint8_t szk = sizeof(key);
	int block_len = (data_len / szk) * szk;
	if (data_len % szk > 0) block_len += szk;
	return block_len;
}

/* -----------------------------
 * Verschlüsseln data -> crypted
 * crypted  muss auf erforderliche Blocksize durch aufrufer initialisiert worden sein
 * data_len: tatsächliche Länge der Eingangsdaten / sizeof(data)
 * key: pointer
 * key_len: Schlüssellänge in Bytes / sizeof(key)
*/
int cbc_encrypt(uint8_t *data, uint8_t *crypted, int data_len, uint8_t *key, int key_len) {
	uint8_t iv_crypt[] = {AES_IV};
	int block_sz = get_blocksize(data_len);

	uint8_t inbuf[block_sz];
	bzero(inbuf, block_sz);
	memcpy(inbuf, data, data_len);

	esp_aes_context ctx;
	esp_aes_init(&ctx);
	esp_aes_setkey(&ctx, key, key_len*8);
	esp_aes_crypt_cbc( &ctx, ESP_AES_ENCRYPT, block_sz, iv_crypt, inbuf, crypted );
	esp_aes_free( &ctx );

	return block_sz;
}


int wiog_encrypt_data(uint8_t* data, uint8_t* encrypted, uint16_t len, uint32_t fid) {
	uint8_t key[] = {AES_KEY};	//CBC-AES-Key
	uint8_t szk = sizeof(key);
	// Frame-ID => 4 letzten Bytes im Key - salted key
	key[szk-4] = (uint8_t)fid;
	key[szk-3] = (uint8_t)(fid>>=8);
	key[szk-2] = (uint8_t)(fid>>=8);
	key[szk-1] = (uint8_t)(fid>>=8);
	return cbc_encrypt(data, encrypted, len, key, szk);
}

/* -----------------------------
 * Entschlüsseln crypted -> data
 * data  muss auf erforderliche Blocksize durch aufrufer initialisiert worden sein
 * crypt_len: tatsächliche Länge (blocksize) der verschlüsselten Eingangsdaten / sizeof(crypted)
 * key: pointer
 * key_len: Schlüssellänge in Bytes / sizeof(key)
*/
int cbc_decrypt(uint8_t *crypted, uint8_t *data, int crypt_len, uint8_t *key, int key_len){
	bzero(data, crypt_len);
	uint8_t iv_decrypt[] = {AES_IV};
	esp_aes_context ctx;
	esp_aes_init(&ctx);
	esp_aes_setkey(&ctx, key, key_len*8);
	int res = esp_aes_crypt_cbc( &ctx, ESP_AES_DECRYPT, crypt_len, iv_decrypt, crypted, data );
	esp_aes_free( &ctx );
	return res;
}

// ----------------------------
// Entschlüsseln eines Datenblockes encryptes
// Daten danach in data, Puffer stellt Aufrufer zur Verfügung
// len Länge des encrypted Block
// Frame-ID wird im System zum salze des Key genutzt
// Result = 0 bei Erfolg
int wiog_decrypt_data(uint8_t* encrypted, uint8_t* data, uint16_t len, uint32_t fid) {
	uint8_t key[] = {AES_KEY};	//CBC-AES-Key
	uint8_t szk = sizeof(key);
	// Frame-ID => 4 letzten Bytes im Key - salted key
	key[szk-4] = (uint8_t)fid;
	key[szk-3] = (uint8_t)(fid>>=8);
	key[szk-2] = (uint8_t)(fid>>=8);
	key[szk-1] = (uint8_t)(fid>>=8);
	return cbc_decrypt(encrypted, data, len, key, szk);
}


// -------------------------------------------------------------------------------------------------------

//uid aus Wifi-efuse-MAC berechnen Wifi-Interface initialisiert
uint16_t get_uid() {
    uint8_t umac[6];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, umac);
    return crc16_be(0, (uint8_t*) umac, 6);
}

//Grundgerüst eines WIOG-Headers
//To / From
//weitere Anpassungen durch Aufrufer erforderlich
wiog_header_t wiog_get_dummy_header(uint8_t mac_to, uint8_t mac_from) {
	wiog_header_t hdr = dummy_header;
	memcpy(&hdr.mac_to,   &mac_net, 5);
	memcpy(&hdr.mac_from, &mac_net, 5);
	memcpy(&hdr.mac_net , &mac_net, 6);
	hdr.mac_to[5] = mac_to;
	hdr.mac_from[5] = mac_from;
	return hdr;
}


// --- CRC16 IoG ----------------------------------------------------------------------------------------
 //                                      16   12   5
 // this is the CCITT CRC 16 polynomial X  + X  + X  + 1.
 // This works out to be 0x1021, but the way the algorithm works
 // lets us use 0x8408 (the reverse of the bit pattern).  The high
 // bit is always assumed to be set, thus we only use 16 bits to
 // represent the 17 bit value.

#define POLY 0x8408
uint16_t crc16(const uint8_t *data_p, uint16_t length) {
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




// --- NVS --------------------------------------------------------------

// Kennzeichen f. Initialisierung NVS, wird bei 1.Aufruf ESP_OK gesetzt
esp_err_t status_nvs = ESP_ERR_INVALID_STATE;

esp_err_t init_nvs()	//ca 35ms !!!
{
	if (status_nvs == ESP_OK) return status_nvs; //nur 1x je Zyklus

	status_nvs = nvs_flash_init();
	if (status_nvs == ESP_ERR_NVS_NO_FREE_PAGES || status_nvs == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK( nvs_flash_erase() );
		status_nvs = nvs_flash_init();
	}
	return status_nvs;
}

//lesen des zuletzt gespeicherten Wifi-Kanals
uint8_t nvs_get_wifi_channel()	// < 1ms
{
	uint8_t res = 0;
	nvs_handle hnvs;
	if (nvs_open("storage", NVS_READONLY, &hnvs) == ESP_OK)
	{
		if (nvs_get_u8(hnvs, "wifi_ch", &res) !=  ESP_OK) res = 0;
		nvs_close(hnvs);
	}
	return res;
}

//schreiben des aktuellen Wifi-Kanals
void nvs_set_wifi_channel(uint8_t ch) // 3 ms
{
	nvs_handle hnvs;
	if (nvs_open("storage", NVS_READWRITE, &hnvs) == ESP_OK)
	{
		nvs_set_u8(hnvs, "wifi_ch", ch);
		nvs_close(hnvs);
	}
}

int32_t nvs_get_sysvar(uint8_t ix)
{
	int32_t res = 0;
	nvs_handle hnvs;
	char c[10];
	sprintf(c, "sysvar%d", ix);
	if (nvs_open("storage", NVS_READONLY, &hnvs) == ESP_OK){
		if (nvs_get_i32(hnvs, c, &res) !=  ESP_OK) res = 0;
		nvs_close(hnvs);
	}
	return res;
}

void nvs_set_sysvar(uint8_t ix, int32_t value) // 3 ms
{
	nvs_handle hnvs;
	char c[10];
	sprintf(c, "sysvar%d", ix);
	if (nvs_open("storage", NVS_READWRITE, &hnvs) == ESP_OK)
	{
		nvs_set_i32(hnvs, c, value);
		nvs_close(hnvs);
	}
}

// ------------------------------------------------------------------------

// Time (Debug)
#ifdef DEBUG_X
int now(){
	return esp_timer_get_time() / 1000;
}

void compare_set_get_tx_power() {
	int8_t max = MAX_TX_POWER;
	while (max >= MIN_TX_POWER) {
		ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(max));
		vTaskDelay(10*MS);
		int8_t maxpwr;
		esp_wifi_get_max_tx_power(&maxpwr);
		printf("Set Tx-Power: %.2f dBm, %d->%d\n", maxpwr *0.25, max, maxpwr);
		max--;
	}
}
#endif
