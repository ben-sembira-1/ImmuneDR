#!/bin/bash
NO_TMUX_ERROR=1
SUCCESS_EXIT_CODE=0

TMUX_SESSION_NAME="gazebo-simulation"

hash tmux > /dev/null 2>&1
if [ $? -ne $SUCCESS_EXIT_CODE ]
then
	echo "Please install tmux to continue"
	exit $NO_TMUX_ERROR
fi
if [ -z "$TMUX" ]
then
	echo "Starting tmux ..."
	tmux new-session -s $TMUX_SESSION_NAME bash "$(dirname $0)/start_simulation_from_tmux.sh"
else
	bash "$(dirname $0)/start_simulation_from_tmux.sh"
fi
