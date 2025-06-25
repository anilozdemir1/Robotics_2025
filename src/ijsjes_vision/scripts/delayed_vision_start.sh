#!/bin/bash
sleep 2
rosrun ijsjes_vision vision_publisher.py &
sleep 5
exec rosrun ijsjes_vision vision_control.py