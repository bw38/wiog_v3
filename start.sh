#!/bin/bash


PROJ_DIR="$HOME/Projects2/wiog_v3"
gnome-terminal  --working-directory="$PROJ_DIR/Sniffer" --title="Sniffer" -x bash -c '. $HOME/Development/esp32/esp-idf/export.sh; exec bash'
gnome-terminal  --working-directory="$PROJ_DIR/Gateway" --title="Gateway" -x bash -c '. $HOME/Development/esp32/esp-idf/export.sh; exec bash'
gnome-terminal  --working-directory="$PROJ_DIR/Sensor"  --title="Sensor"  -x bash -c '. $HOME/Development/esp32/esp-idf/export.sh; exec bash'
gnome-terminal  --working-directory="$PROJ_DIR/Actor"   --title="Actor"   -x bash -c '. $HOME/Development/esp32/esp-idf/export.sh; exec bash' 

# sshfs pi@testgw: ~/pigw

/home/joerg/Development/esp32/eclipse/eclipse
