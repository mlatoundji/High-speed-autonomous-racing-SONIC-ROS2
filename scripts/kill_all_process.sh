#!/bin/bash

pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f tracker.py 2>/dev/null
pkill -9 -f localplanner.py 2>/dev/null
pkill -9 -f globalplanner.py 2>/dev/null
pkill -9 -f localisation.py 2>/dev/null
pkill -9 -f lap_timer.py 2>/dev/null
pkill -9 -f latency_injector.py 2>/dev/null
pkill -9 -f control_manager.py 2>/dev/null
killall -9 gzserver gzclient gazebo 2>/dev/null
killall -9 robot_state_publisher rviz2 2>/dev/null
ros2 daemon stop
sleep 3


pkill -9 -f bof 2>/dev/null
killall -9 bof 2>/dev/null
pkill -9 -f odom_noise 2>/dev/null
ros2 daemon stop
sleep 2
ros2 daemon start
ros2 node list