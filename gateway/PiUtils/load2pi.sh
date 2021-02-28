#!/bin/bash

# Script zum hochladen der Kommunikationstools auf den raspberry
# Verzeichnisse "/home/pi/esp/bin" müssen auf dem RPi angelegt sein
# evtl esptool-master aktualisieren

# Eclipse erzeugt binaries
# automatischer upload zum pi "~/esp/bin" mit makeit.sh (muss konfiguriert sein)
# manueller upload im RPi
#   cd ~/esp
#   ./espup.sh

echo "./load2pi.sh servername"
GW=$1
rsync -avze ssh  ./UploadTools/*   pi@$GW:~/esp/

