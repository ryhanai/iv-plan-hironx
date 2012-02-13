#!/bin/bash

ROBOTNAME=localhost
RTCTREE_NAMESERVERS=$ROBOTNAME:2809
TERMOPT=--window-with-profile="Default"
gnome-terminal -t "AppRecog" -e ./comup_loadpicture.sh --geometry 80x24+0+530 $TERMOPT &
 sleep 3
./comcon_loadpicture.sh
./comact_loadpicture.sh
