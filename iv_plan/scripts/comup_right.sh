#!/bin/bash

PKGDIR=`rosstack find iv-pkg-unreleased`
echo $PKGDIR
$PKGDIR/CvCapture/CameraCapture/CameraCaptureComp -f rtc_right.conf &
#$PKGDIR/CvCapture/CameraViewer/CameraViewerComp -f rtc_right.conf &
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/prog/hironx/iv-pkg-unreleased/opencv23/lib $PKGDIR/HandRcg/mi_match/build/bin/mi_matchComp -f rtc_right.conf
