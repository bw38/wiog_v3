#!/bin/bash

# Script zur Kommunikation Raspberry ==> ESP32-Plug-Board
# ohne Argument -> Upload der Binaries 
# --erase -> Flash löschen
# -- getid
#
# sonstige Argumente werden als Pfad interprtiert
#   -> nicht erforderlich bei Aufruf von Kommandozeile
#   -> Pfad zum aktuellen Verzeichnis bei Aufruf von Eclipse => ${project_loc}
#   -> andere Pfade sind nicht zulässig
#
# Konfigurationsdaten in board.conf des aktuellen Verzeichnisses


while [ $# -gt 0 ] #Solange die Anzahl der Parameter ($#) größer 0
do
  if [ $1 = "--init" ]; then
    INIT=1
  elif [ $1 = "--erase" ]; then
    ERASE=1
  elif [ $1 = "--getid" ]; then
    GETID=1
  else 
    cd $1   #Pfad zum aktuellen Verzeichnis -> nur von eclipse nowendig
    echo Switch to $1
  fi
  shift     #Parameter verschieben $2->$1, $3->$2, $4->$3,...
done

# Konfigurationsdaten einfügen
TOOLPATH="./esptool-master"
ESPTOOL=$TOOLPATH/esptool.py
PORT="/dev/ttyAMA0"

#ggf miniterm abschießen
pkill -f $PORT

# Raspberry - pin config
# P1.8  - ALT0 - TxD -> prüfen "gpio readall", ggf über raspi-config konfigurieren -> /dev/ttyAMA0
# P1.10 - ALT0 - RxD -> dto
# P1.11 - ALT3 - gpio.0 Chip Reset
# P1.13 - OUT  - gpio.2 Chip program

# alt wiring pi
#gpio mode 0 alt3
#gpio mode 2 out

gpio -fsel 17 alt3  # RTS0
gpio -fsel 27 out   # DTR manuell gesteuert


if [ $GETID ]; then
	echo
	echo "Read SPI-Flash Data:"
	echo "---------------"
    gpio -write 27 0
	$ESPTOOL -p $PORT flash_id
    gpio -write 27 1
	echo
	echo "Read SoC Data:"
	echo "---------------"
    gpio -write 27 0
    $ESPTOOL -p $PORT chip_id
    gpio -write 27 1
    exit
fi


if [ $ERASE ]; then
    gpio -write 27 0
    $ESPTOOL \
        -p $PORT \
        erase_flash
    gpio -write 27 1
    exit
fi


echo
echo Image-Info:
echo -----------
$ESPTOOL --chip esp32 image_info ./bin/wiog_gw_v3.bin
echo
gpio -write 27 0
$ESPTOOL \
    --chip esp32 \
    --port $PORT \
    --baud 921600 \
    --before default_reset \
    --after hard_reset \
    write_flash -u \
    --flash_mode dio \
    --flash_freq 40m \
    --flash_size detect \
    0x1000  ./bin/bootloader.bin \
    0x8000  ./bin/partition-table.bin \
    0x10000 ./bin/wiog_gw_v3.bin

gpio -write 27 1


