#!/bin/bash

t0=$(ip addr | grep can0)
t1=$(ifconfig | grep -o can0)
t2='can0'

if [ -z "$t0" ]; then
  echo "can0 not found"
elif [ "$t1" = "$t2" ]; then
  echo "can0 is up"
else
  echo "can0 is down, bringing up with 500kb/s"
  ip link set can0 type can bitrate 500000
  ip link set dev can0 up
fi

exit
