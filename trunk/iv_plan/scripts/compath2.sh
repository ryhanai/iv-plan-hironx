#!/bin/sh
rtfind / -i $1/$2 | sed "s/$2//g"
