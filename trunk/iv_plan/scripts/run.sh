#!/bin/bash

ROBOTNAME=localhost
RTCTREE_NAMESERVERS=$ROBOTNAME:2809
TERMOPT=--window-with-profile="Default"
# gnome-terminal -t "AppRecogRight" -e ./comup_right.sh --geometry 80x24+0+530 $TERMOPT &
gnome-terminal -t "AppRecogLeft" -e ./comup_left.sh --geometry 80x24+0+530 $TERMOPT &
 sleep 3
./comcon.sh
./comact.sh
