#!/bin/bash

# Gazebo Simulation
# tmux splitw -v gazebo --verbose worlds/iris_arducopter_runway.world
# tmux splitw -c ~/clones/ardupilot/ArduCopter/ -v ../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map --out=udp:localhost:14560
# SITL Simulation
tmux splitw -c ~/clones/ardupilot/ArduCopter/ -v ../Tools/autotest/sim_vehicle.py -v ArduCopter --console --map --out=udp:localhost:14550 --out=udp:localhost:14560

tmux splitw -h
