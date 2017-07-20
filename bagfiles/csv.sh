#!/bin/bash
export f=gpsrlodom.bag
# export topic=/odometry/filtered_map
export topic=/odomCombined
rostopic echo -p -b $f $topic >${f%.bag}_${topic//\//_}.csv
