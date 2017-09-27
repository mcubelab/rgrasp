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
  
  now=$(date +"%m_%d_%Y_%H%M%S")
  { time python -m cProfile -o $APCDATA_BASE/learning/fullLogging/pick-succ-$now.cprof learned_heuristic.py -v file -f -k ; } \
    2> $APCDATA_BASE/learning/fullLogging/pick-succ-time-$now.txt \
    | tee $APCDATA_BASE/learning/fullLogging/pick-succ-$now.txt 
    #
  # #kill %1
  # 
  now=$(date +"%m_%d_%Y_%H%M%S")
  { time python -m cProfile -o $APCDATA_BASE/learning/fullLogging/pick-$now.cprof learned_heuristic.py -v file -k ; } \
    2> $APCDATA_BASE/learning/fullLogging/pick-time-$now.txt \
    | tee $APCDATA_BASE/learning/fullLogging/pick-$now.txt 

  #kill %1
  
  now=$(date +"%m_%d_%Y_%H%M%S")
  #time python -m cProfile -o $APCDATA_BASE/learning/fullLogging/stow-succ-$now.cprof placing16.py -v file -f -k | tee $APCDATA_BASE/learning/fullLogging/stow-succ-$now.txt &
  
  { time python -m cProfile -o $APCDATA_BASE/learning/fullLogging/stow-succ-$now.cprof placing16.py -v file -k -f ; } \
    2> $APCDATA_BASE/learning/fullLogging/stow-succ-time-$now.txt \
    | tee $APCDATA_BASE/learning/fullLogging/stow-succ-$now.txt 
    
  
  now=$(date +"%m_%d_%Y_%H%M%S")
  
  { time python -m cProfile -o $APCDATA_BASE/learning/fullLogging/stow-$now.cprof placing16.py -v file -k ; } \
    2> $APCDATA_BASE/learning/fullLogging/stow-time-$now.txt \
    | tee $APCDATA_BASE/learning/fullLogging/stow-$now.txt 

  # kill %1
  #kill %1
done
