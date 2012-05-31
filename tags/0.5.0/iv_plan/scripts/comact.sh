#!/bin/bash
source /usr/local/share/rtshell/shell_support

func_act ()
{
    comppath=`./compath2.sh $1 AppRecog0.rtc`
    echo $comppath
    rtcwd $comppath
    rtact CaptureCamera0.rtc AppRecog0.rtc
}

func_act rhand_cxt
func_act lhand_cxt
