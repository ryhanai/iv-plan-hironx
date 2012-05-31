#!/bin/bash

PKGDIR=`rosstack find iv-pkg-unreleased`
echo $PKGDIR
#$PKGDIR/SequencePlayerService/seq_service_socket.py hiro014 &
$PKGDIR/LoadPictureComp/LoadPictureComp -f ./rtc.conf &

LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/prog/hironx/iv-pkg-unreleased/opencv23/lib $PKGDIR/app-recog/build/bin/AppRecogComp -f ./rtc.conf
