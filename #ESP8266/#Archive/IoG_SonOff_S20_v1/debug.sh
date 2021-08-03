#!/bin/bash

# serial debug

# Konfigurationsdaten einfügen
source ./board.conf

pkill -f $PORT
python3 -m serial.tools.miniterm $PORT 74880 --rts 0 --dtr 0 | tr -d '\r'
