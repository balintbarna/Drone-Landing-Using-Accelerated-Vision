#!/bin/bash

# sudo -su root script -c bash "screen -S detect -d -m bash -c 'python3 -m dai_jupyter.nonb.cam_detect_target; exec'"
# echo "started detect"
# sleep 5
screen -S core -d -m bash -c 'source live.bash; livesrc; roscore; exec'
echo "started core"
sleep 5
screen -S bag -d -m bash -c 'source live.bash; livesrc; cd ~/bags; rosbag record /state_machine/state /landing_pos_error/local_frame /mavros/setpoint_position/local /mavros/local_position/pose /mavros/state; exec'
echo "started bag"
sleep 5
screen -S launch -d -m bash -c 'source live.bash; livesrc; roslaunch dai_live live.launch; exec'
echo "started launch"
