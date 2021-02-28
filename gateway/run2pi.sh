#!/bin/bash

# Programmnamen und Pfade idf.py biuld übernommen !

PROJECTNAME="wiog_gw_v3"    # ./CMakeLists.txt
BINPATH="./build"

SERVER="testgw"

idf.py build

scp $BINPATH/bootloader/bootloader.bin           pi@$SERVER:/home/pi/esp/bin/bootloader.bin
scp $BINPATH/partition_table/partition-table.bin pi@$SERVER:/home/pi/esp/bin/partition-table.bin
scp $BINPATH/wiog_gw_v3.bin                      pi@$SERVER:/home/pi/esp/bin/wiog_gw_v3.bin

# sleep 3

# s. idf.py build
#0x1000 build/bootloader/bootloader.bin 
#0x8000 build/partition_table/partition-table.bin 
#0x10000 build/wiog_gw_v3.bin


