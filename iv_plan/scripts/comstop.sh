#!/bin/bash
source /usr/local/share/rtshell/shell_support

func_stop ()
{
    comppath=`./compath2.sh $1 CameraCapture0.rtc`
    echo $comppath
    rtcwd $comppath
    rtdeact CameraCapture0.rtc
}

func_stop rhand_cxt
func_stop lhand_cxt
