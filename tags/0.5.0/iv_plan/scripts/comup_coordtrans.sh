#!/bin/bash

PKGDIR=`rospack find iv_plan`
echo $PKGDIR
(cd $PKGDIR/src; ./CoordTransComp.py -f ./rtc.conf)
