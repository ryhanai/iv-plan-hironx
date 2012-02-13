#!/bin/bash

ROBOTNAME=localhost
RTCTREE_NAMESERVERS=$ROBOTNAME:2809
TERMOPT=--window-with-profile="Default"

USE_HIROIF=0
USE_HANDCAMRTC=1
USE_COORDTRANSRTC=0

while getopts rc OPT
do
    case $OPT in
	"r") USE_HANDCAMRTC=1 ;;
	"t") USE_COORDTRANSRTC=1 ;;
	"c") USE_HIROIF=1 ;;
	"a") USE_ARPOSE=1 ;;
    esac
done

if [ $USE_HIROIF == 1 ]; then
    gnome-terminal -t "HiroNXInterface" -e ./comup_hironxif.sh --geometry 80x24+0+530 $TERMOPT &
fi

gnome-terminal -t "AppRecogRight" -e ./comup_right.sh --geometry 80x24+0+530 $TERMOPT &
gnome-terminal -t "AppRecogLeft" -e ./comup_left.sh --geometry 80x24+0+530 $TERMOPT

./comwait.sh lhand_cxt AppRecog0.rtc
./comwait.sh rhand_cxt AppRecog0.rtc

./comcon.sh
./comact.sh
