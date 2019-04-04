#! /usr/bin/env bash

echo "Hello!"
echo "Compiling and configuring Pixhawk1, this might take a while"

#echo "1. configure board Pixhawk1"
#./waf configure --board Pixhawk1
echo "1. configure board Pixhawk2 CubeBlack" 
./waf configure --board CubeBlack # TODO: check param for Pixhawk2

echo "2. build code"
./waf copter                        # build code

#if --beep or -b is added as argument for this script: beep (annoying sound^^)

if [ $0 = "--beep" ]; then
    beep
fi

echo "3. upload arducopter build to pixhawk"

./waf --targets bin/arducopter --upload
