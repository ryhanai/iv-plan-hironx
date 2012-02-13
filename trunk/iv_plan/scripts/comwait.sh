#!/bin/bash
source /usr/local/share/rtshell/shell_support

while :
do
    COMPATH=`./compath2.sh $1 $2`
    rtcwd $COMPATH
    COMSTAT=`rtcat $2 | head -1 | wc -l`
    # COMSTAT=`rtcat $COMPATH | head -1 | sed -e "s/$1 *//"`
    # echo "$1: $COMSTAT"
    if [ $COMSTAT = 1 ]; then
    	break
    else
	sleep 1
    fi
done
