#!/bin/bash

source ./board.conf

# Argument möglich, z.B clean

# Bestands-Binaries löschen
if [ "$1" = "clean" ]; then
    rm ./bin/eagle*
fi

rm ../bin/eagle*

# -B -> force build
make $1 $2 -B COMPILE=gcc BOOT=$boot APP=$app SPI_SPEED=$spi_speed SPI_MODE=$spi_mode SPI_SIZE_MAP=$spi_size_map

mv ../bin/eagle* ./bin
# nur Raspberry upload, ggf anpassen
#scp ./bin/*.bin pi@raspi5:/home/pi/esp/bin

echo ----------------------
echo "size of:"
printf "IRAM: "
stat --printf="%s\n" ./bin/eagle.flash.bin
printf "IROM: "
stat --printf="%s\n" ./bin/eagle.irom0text.bin
echo ----------------------
