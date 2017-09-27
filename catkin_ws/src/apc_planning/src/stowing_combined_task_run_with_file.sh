#!/bin/bash
#
#  Flags available in planner17.py:
#
#  -v   Selects visition. Options: real or file. Default: real
#  -p   Adds Pause to the system. Default: False
#  -n   Determines if motions are executed. Default: True
#  -f   Forces planning to always succede. Default: False
#       To force success on trying the last strategy in the strategy list; This is useful for testing system in virtual environment.
#  -j   Provides json filename with the objects in tote and storage system. Options: filename.json Default='apc_stow_task17.json'
#  -a   List of primitives used. Option:subset of suction-tote, grasp-tote or all. Default='all'
#  -t   Themes that the debbuger should display. Options:Subset of hardware, planner, manipulation, debug or all. Default='all'
#  -l   Level of verbosity of the debugger. Options: from 0 (silenece) to 5 (all). Default='5'
#  -e   Determines if the planner is running in the experiment mode meaning that it will try to collect data
#  -m   Allow a human in the loop to take several decisions such as what object have been pick. This flag only makes sense in the experiment mode


###CHANGE JSON FILE #####
#
# Change the file name that goes after -j in the next lines (it can be named 'item_location_file_X.json' where X should be an appropiate name). Make sure the json file is stored in arc/input.
#
###


time=$(date +%s)

#
# Run stow as in contest but with more time (1h)
#
#script -q -c "python -m pdb planner17.py -j stow_pick_item_location_32_objects_0_empty_bins.json -t 'all' -l 5  -d baseline --duration 3600 --combined " | tee -a $ARC_BASE/output/planner_log_$time.txt

#script -q -c "python -m pdb planner17.py -j stow_pick_item_location_32_objects_0_empty_bins.json -t 'all' -l 5  -d baseline --duration 3600 --combined -a 'suction-tote'" | tee -a $ARC_BASE/output/planner_log_$time.txt
#script -q -c "python -m pdb planner17.py -j stow_pick_item_location_32_objects_04.json -t 'all' -l 5  -d baseline --duration 3600 --combined " | tee -a $ARC_BASE/output/planner_log_$time.txt
script -q -c "python -m pdb planner17.py -j item_location_file.json -t 'all' -l 5  -d baseline --duration 3600 --combined --interactive interactive_result.json" | tee -a $ARC_BASE/output/planner_log_$time.txt


