#!/bin/bash

#script for collision suppervision

for i in `seq 1 100`;
do
  echo $i
  rosservice call -- /robot1_SetCartesian  500 675 215 0.0464 0.5958 -0.8 0.0546
done
