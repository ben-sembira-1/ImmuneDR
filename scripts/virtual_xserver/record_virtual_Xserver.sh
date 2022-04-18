#! /bin/bash
NOW=$(date +"date%d-%m-%y_time%H-%M-%S")
ffmpeg -f x11grab -framerate 5 -video_size 1920x1080 -i :99.0 /home/pilot/app/system_tests/"$NOW"tests_screen_record.mp4
