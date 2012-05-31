#!/bin/bash

PKGDIR=`rospack find HiroNXInterface`
echo $PKGDIR
$PKGDIR/HiroNXProvider/HiroNXProvider.py -f ./rtc.conf &
$PKGDIR/HiroNXGUI/WxHiroNXGUI.py -f ./rtc.conf
