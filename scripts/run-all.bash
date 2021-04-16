#!/bin/bash

sudo screen -S detect -d -m bash -c 'python3 -m dai_jupyter.nonb.cam_detect_target; exec'
echo "started detect"
sleep 5
screen -S core -d -m bash -c 'source live.bash; livesrc; roscore; exec'
echo "started core"
sleep 5
screen -S bag -d -m bash -c 'source live.bash; livesrc; cd ~/bags; rosbag record -O subset /landing_pos_error/transformed /mavros/local_position/pose; exec'
echo "started bag"
sleep 5
screen -S launch -d -m bash -c 'source live.bash; livesrc; roslaunch dai_live live.launch; exec'
echo "started launch"
