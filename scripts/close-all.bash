#!/bin/bash

screen -S launch -X at '#' stuff ^C
echo "stopped launch"
sleep 10
screen -S bag -X at '#' stuff ^C
echo "stopped bag"
sleep 3
screen -S core -X at '#' stuff ^C
echo "stopped core"