#!/bin/bash
source /usr/local/share/rtshell/shell_support

func_restart ()
{
    comppath=`./compath2.sh $1 CameraCapture0.rtc`
    echo $comppath
    rtcwd $comppath
    rtact CameraCapture0.rtc
}

func_restart rhand_cxt
func_restart lhand_cxt
