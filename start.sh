#!/bin/bash


PROJ_DIR="$HOME/Projects2/wiog_v3"

export W0=118
export H0=29
export L1=0
export L2=960
export T1=0
export T2=520

gnome-terminal --window --geometry=${W0}x${H0}+${L1}+${T1} --working-directory="$PROJ_DIR/sniffer" --title="Sniffer" -x bash -c '. $HOME/Development/esp32/esp-idf/export.sh; exec bash'
gnome-terminal --window --geometry=${W0}x${H0}+${L1}+${T2} --working-directory="$PROJ_DIR/gateway" --title="Gateway" -x bash -c '. $HOME/Development/esp32/esp-idf/export.sh; exec bash'
gnome-terminal --window --geometry=${W0}x${H0}+${L2}+${T1} --working-directory="$PROJ_DIR/sensor"  --title="Sensor"  -x bash -c '. $HOME/Development/esp32/esp-idf/export.sh; exec bash'
gnome-terminal --window --geometry=${W0}x${H0}+${L2}+${T2} --working-directory="$PROJ_DIR/actor"   --title="Actor"   -x bash -c '. $HOME/Development/esp32/esp-idf/export.sh; exec bash' 

# sshfs pi@testgw: ~/pigw

#/home/joerg/Development/esp32/eclipse/eclipse
