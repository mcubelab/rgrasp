#!/bin/bash

for i in `seq 1 1`;
do
  cd $APC_BASE
  git pull -X theirs
  cd $APC_BASE/catkin_ws
  catkin_make

  mkdir -p $APCDATA_BASE/learning/fullLogging
  #recordmydesktop &
  cd $APC_BASE/catkin_ws/src/apc_planning/src
  
  rosrun apc_planning pose_generator_stow.py
  
  now=$(date +"%m_%d_%Y_%H%M%S")
  
  cp apc_stow_task.json $APCDATA_BASE/learning/fullLogging/apc_stow_task_$now.json
  cp apc_stow_task.pose.json $APCDATA_BASE/learning/fullLogging/apc_stow_task_$now.pose.json
  
  
  { time python -m cProfile -o $APCDATA_BASE/learning/fullLogging/stow-succ-$now.cprof placing16.py -v file -k -f -j apc_stow_task.json ; } \
    2> $APCDATA_BASE/learning/fullLogging/stow-succ-time-$now.txt \
    | tee $APCDATA_BASE/learning/fullLogging/stow-succ-$now.txt 
    
  
  now=$(date +"%m_%d_%Y_%H%M%S")
  
  cp apc_stow_task.json $APCDATA_BASE/learning/fullLogging/apc_stow_task_$now.json
  cp apc_stow_task.pose.json $APCDATA_BASE/learning/fullLogging/apc_stow_task_$now.pose.json
  
  
  { time python -m cProfile -o $APCDATA_BASE/learning/fullLogging/stow-$now.cprof placing16.py -v file -k -j apc_stow_task.json ; } \
    2> $APCDATA_BASE/learning/fullLogging/stow-time-$now.txt \
    | tee $APCDATA_BASE/learning/fullLogging/stow-$now.txt 
  
done
