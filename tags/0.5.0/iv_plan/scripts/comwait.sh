#!/bin/bash
source /usr/local/share/rtshell/shell_support

while :
do
    COMPATH=`./compath.sh $1`
    echo $COMPATH
    rtcwd $COMPATH
    COMSTAT=`rtcat $1 | head -1 | wc -l`
    if [ $COMSTAT = 1 ]; then
    	break
    else
	sleep 1
    fi
done
