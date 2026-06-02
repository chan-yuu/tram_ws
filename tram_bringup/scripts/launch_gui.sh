#!/bin/bash
export DISABLE_ROS1_EOL_WARNINGS=1
source /home/cyun/Documents/project/tram_ws/devel/setup.bash
python3 /home/cyun/Documents/project/tram_ws/src/tram_bringup/scripts/scenario_gui.py
