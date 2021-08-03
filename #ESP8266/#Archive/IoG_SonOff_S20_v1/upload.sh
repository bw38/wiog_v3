#!/bin/bash

# Script zur Kommunikation mit dem Flash auf dem ESP-Board
# ohne Argument -> Upload der Binaries ../bin/eagle.flash.bin und ../bin/eagle.irom0text.bin
# --erase -> Flasch löschen
# --init  -> Upload der Binaries ../bin/esp_init_data_default.bin und ../bin/blank.bin
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
source ./board.conf


#ggf miniterm abschießen
pkill -f $PORT


if [ $GETID ]; then
	echo
	echo "Read SPI-Flash Data:" 
	echo "---------------"    
	${TOOLPATH}/esptool.py -p $PORT flash_id
	echo
	echo "Read SoC Data:" 
	echo "---------------"	
    ${TOOLPATH}/esptool.py -p $PORT chip_id    
    exit
fi 


if [ $ERASE ]; then
    ${TOOLPATH}/esptool.py \
        -p $PORT \
        erase_flash
    exit
fi    

if [ $INIT ]; then
    ${TOOLPATH}/esptool.py \
        -p $PORT \
        -b $BR \
        -a $RST \
        write_flash \
        -fm $spi_mode \
        -fs $FS \
        $ADR_INIT  ./bin/esp_init_data_default.bin \
        $ADR_BLANK ./bin/blank.bin
    exit
fi

echo
echo Image-Info: eagle.flash.bin
echo ----------------------------
${TOOLPATH}/esptool.py image_info ./bin/eagle.flash.bin
echo


${TOOLPATH}/esptool.py \
    -p $PORT \
    -b $BR \
    -a $RST \
    write_flash \
    -fm $spi_mode \
    -fs $FS \
    0x00000 ./bin/eagle.flash.bin \
    0x10000 ./bin/eagle.irom0text.bin


# serial debug
pkill -f $PORT
python3 -m serial.tools.miniterm $PORT 74880 --rts 0 --dtr 0 | tr -d '\r'
# RTS -> CH_PD und DTR -> GPOI0 werden mit dem Wert 0 auf High gesetzt !!!
# CR löschen, sonst zusätzliche Leerzeile in der Ausgabe
