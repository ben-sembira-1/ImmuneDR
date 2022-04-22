#! /bin/bash
Xvfb :99 -screen 0 1920x1080x24 &
/bin/bash /home/pilot/screen_record/record_virtual_Xserver.sh
