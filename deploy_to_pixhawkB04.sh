#! /usr/bin/env bash

echo "Hello!"

if [ $1 = "-2" ]; then
    echo "1. configure board Pixhawk2 CubeBlack" 
    ./waf configure --board CubeBlack
else 
    echo "1. configure board Pixhawk1"
    ./waf configure --board Pixhawk1
fi

echo "Compiling Pixhawk, this might take a while"

#echo "1. configure board Pixhawk1"
#./waf configure --board Pixhawk1
#echo "1. configure board Pixhawk2 CubeBlack" 
#./waf configure --board CubeBlack # TODO: check param for Pixhawk2

echo "2. build code"
./waf copter                        # build code

##if --beep or -b is added as argument for this script: beep (annoying sound^^)

#if [ $0 = "--beep" ]; then
#    beep
#fi

echo "3. upload arducopter build to pixhawk"

./waf --targets bin/arducopter --upload
