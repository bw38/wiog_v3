#ifndef BME280_SPI_IF_H_
#define BME280_SPI_IF_H_


#include "bme280_defs.h"
#include "bme280.h"

//externe Funktionen
extern esp_err_t bme280_spi_master_init(bool wu);
extern void bme280_add_entries();

#endif
