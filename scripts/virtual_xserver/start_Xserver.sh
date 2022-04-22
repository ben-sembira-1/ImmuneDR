#! /bin/bash
# trap wait_and_exit sigint
# function wait_and_exit() {
#     echo "Closing..."
#     sleep 3
#     echo "Stopping!"
#     exit
# }
Xvfb :99 -screen 0 1920x1080x24
