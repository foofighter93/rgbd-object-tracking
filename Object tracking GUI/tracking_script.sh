#!/bin/bash
# My first script
echo "FIrst arg: $1"
cd ~/catkin_ws/ && source devel/setup.bash && catkin_make && rosrun objtrack tracking cloud:=camera/depth/points _filepath:=$1

