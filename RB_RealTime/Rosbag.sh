#!/bin/bash

clear
echo "  "
echo "  "
read   -p    "Enter the Rosbag file name :  "  filename
read   -p    "Enter duration in sec ( 0 for unlimited) : "  duration
mkdir $filename
cp all_sub  ./$filename
cd $filename

if [ $duration -eq 0 ];
then 
set -x
./all_sub &
P1=$!
rosbag record   /camera/cam_1     /camera/cam_2    /camera/cam_3   /radar   /radar_bf  /pandar40p  -O  $filename  &
P2=$!
else
set -x
./all_sub $duration & 
P1=$!
rosbag record  /camera/cam_1     /camera/cam_2    /camera/cam_3   /radar   /radar_bf  /pandar40p   -O  $filename  --duration=$duration  &
P2=$!
fi

wait $P1 


	