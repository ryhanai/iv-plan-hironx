#!/bin/bash

PKGDIR=`rosstack find iv-pkg-unreleased`
echo $PKGDIR
$PKGDIR/CameraComp/CaptureCameraComp -f rtc_right.conf &
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/prog/hironx/iv-pkg-unreleased/opencv23/lib $PKGDIR/app-recog/build/bin/AppRecogComp -f rtc_right.conf
