#!/bin/bash
export f=circlerlfusion.bag
# export topic=/odometry/filtered_map
export topic=/odomCombined
export topic1=/odometry/filtered_map
rostopic echo -p -b $f $topic >${f%.bag}_${topic//\//_}.csv
