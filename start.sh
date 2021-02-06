#!/bin/bash


PROJ_DIR="$HOME/Projects2/wiog_v3"
gnome-terminal  --working-directory="$PROJ_DIR/sniffer" --title="Sniffer" -x bash -c '. $HOME/Development/esp32/esp-idf/export.sh; exec bash'
gnome-terminal  --working-directory="$PROJ_DIR/gateway" --title="Gateway" -x bash -c '. $HOME/Development/esp32/esp-idf/export.sh; exec bash'
gnome-terminal  --working-directory="$PROJ_DIR/sensor"  --title="Sensor"  -x bash -c '. $HOME/Development/esp32/esp-idf/export.sh; exec bash'
gnome-terminal  --working-directory="$PROJ_DIR/actor"   --title="Actor"   -x bash -c '. $HOME/Development/esp32/esp-idf/export.sh; exec bash' 

# sshfs pi@testgw: ~/pigw

/home/joerg/Development/esp32/eclipse/eclipse
