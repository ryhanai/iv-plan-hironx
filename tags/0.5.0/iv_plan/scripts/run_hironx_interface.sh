#!/bin/bash
source /usr/local/share/rtshell/shell_support

ROBOTNAME=hiro014
RTCTREE_NAMESERVERS=$ROBOTNAME:2809
TERMOPT=--window-with-profile="Default"

USE_HIROIF=0
USE_HANDCAMRTC=1
USE_COORDTRANSRTC=0

gnome-terminal -t "HiroNXInterface" -e ./comup_hironxif.sh --geometry 80x24+0+530 $TERMOPT &

./comwait.sh HiroNXProvider0.rtc

compath=`./compath.sh HiroNXGUI0.rtc`
rtcwd $compath

rtcon HiroNXProvider0.rtc:HiroNX HiroNXGUI0.rtc:HiroNX
rtcon HiroNXProvider0.rtc:HIRO HiroNXGUI0.rtc:HIRO

rtact HiroNXProvider0.rtc
rtact HiroNXGUI0.rtc
