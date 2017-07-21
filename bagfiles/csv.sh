#!/bin/bash
export topic=/odomCombined
export topic1=/odometry/filtered_map
for f in *.bag;
do
  rostopic echo -p -b $f $topic >${f%.bag}_${topic//\//_}.csv
  rostopic echo -p -b $f $topic1 >${f%.bag}_${topic1//\//_}.csv
done
