

#!/bin/bash

time=$(date +%s)


# Run grasping as in contest with grasping and flush
#
script -q -c "python -m pdb planner_grasp.py" | tee -a $CODE_BASE/output/planner_log_$time.txt
