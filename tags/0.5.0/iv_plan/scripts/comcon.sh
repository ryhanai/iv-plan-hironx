#!/bin/bash
source /usr/local/share/rtshell/shell_support

func_con ()
{
    comppath=`./compath2.sh $1 AppRecog0.rtc`
    rtcwd $comppath
    rtcon CaptureCamera0.rtc:CameraImage AppRecog0.rtc:InputImage
}

func_con rhand_cxt
func_con lhand_cxt
