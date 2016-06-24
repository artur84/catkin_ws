#!/bin/bash								

#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#
# this file sets up some important environmental variables 
# and the wireless connection to iteshu-robot-net network in order to 
# be able to run a demo with the iteshu-robot using several 
# interconected computers
##########################################################################
echo # Drop down a line before we start for readability
echo "Setup wheelchair demo network connections: Created by Arturo Escobedo."
echo "INFO: If you want to keep ROS_MASTER_URI env variables in global bash shell"
echo "This script should be run as $ . ./catkin_ws/src/iteshu_robot/conf/scripts/setdemo.sh"
echo "ENJOY!"; echo;
# Check that we're in a BASH shell
if test -z "$BASH" ; then
  echo "setdemo.sh must be run in the BASH shell... Aborting."; echo;
  exit 192
fi
    # Export ROS configuration for iteshu-robot-net network
    export ROS_MASTER_URI=http://robot_demo:11311
    echo "ROS_MASTER_URI="; echo $ROS_MASTER_URI; echo;
    export ROS_HOSTNAME=$(hostname)_demo  #Edit this (hostname) at /etc/hostname file 
    echo "ROS_HOSTNAME="; echo $ROS_HOSTNAME; echo;

#To check if 
network_state=$(nmcli con status | grep -c iteshu-robot-net)

#if we are not  connected to iteshu-robot-net then connect
if test $network_state -eq 0; then
    # Connect to iteshu-robot-net network
    echo "Connecting to iteshu-robot-net wireless network"; echo;
    nmcli con up id iteshu-robot-net --timeout 10.0
fi  

# Exit clean
echo "setdemo.sh finished."; echo;
#exit 0
