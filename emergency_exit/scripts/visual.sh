#!/bin/bash

source /opt/ros/lunar/setup.bash
source /home/micha/Workspaces/ros_cps/devel/setup.bash
export ROSCONSOLE_FORMAT='[${node}:${line}]: ${message}'
roslaunch emergency_exit small_maze.launch visual:=true

