

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

time=$(date +%s)

#
# Run suction with a given set of objects that in the file: apc_stow_task
#
# NOT YET AVAILABLE
# script -q -c "python planner17.py -j apc_stow_task17.json -a 'suction-tote' -t 'all' -l 5 -e -m" | tee -a $ARC_BASE/output/planner_log_$time.txt

#
# Run grasping with a given set of objects that in the file: apc_stow_task
#
# script -q -c "python planner17.py -j apc_stow_task17.json -a 'suction-tote' -t 'all' -l 5 -e" | tee -a $ARC_BASE/output/planner_log_$time.txt

#
# Run grasping with a given set of objects that in the file: apc_stow_task
#
# script -q -c "python planner17.py -j apc_stow_task17.json -a 'grasp-tote' -t 'all' -l 5 -e -m -n" | tee -a $ARC_BASE/output/planner_log_$time.txt

#
# Run suction to collect experiments and data. 20 objects are randomly selected from the 40 identities available
#
# script -q -c "python  planner17.py -j apc_stow_task_experiment.json -a 'suction-tote' -t 'all' -l 5 -e -m -d experiment" | tee -a $ARC_BASE/output/planner_log_$time.txt


#
# Run grasping to collect experiments and data. 20 objects are randomly selected from the 40 identities available
#
# script -q -c "python  planner17.py -j apc_stow_task_experiment.json -a 'grasp-tote' -t 'all' -l 5 -e -m -d experiment" | tee -a $ARC_BASE/output/planner_log_$time.txt
# script -q -c "python planner17.py -j apc_stow_task_experiment.json -a 'grasp-tote' -t 'all' -l 5 -e -m" | tee -a $ARC_BASE/output/planner_log_$time.txt


#
# Run test in virtual
#
#script -q -c "python -m pdb  planner17.py -j item_location_file.json -t 'all' -l 5 -v virtual -d random -s" | tee -a $ARC_BASE/output/planner_log_$time.txt
#python -m cProfile -o $ARC_BASE/output/planner_prof_$time.txt planner17.py -j item_location_file.json -t 'all' -l 5 -v virtual -d random -s
#python -m pdb planner17.py -j item_location_file.json -t 'all' -l 5 -v virtual -d random -s

#
# Run stow as in contest
#
#script -q -c "python -m pdb planner17.py -j item_location_file.json -t 'all' -l 5  -d baseline --duration 900 -a 'suction-tote' " | tee -a $ARC_BASE/output/planner_log_$time.txt
#python -m cProfile -o $ARC_BASE/output/planner_prof_$time.txt planner17.py -j item_location_file.json -t 'all' -l 5  -d baseline --duration 900 


#
# Run picking in virtual
#
#script -q -c "python -m pdb planner17.py -j item_location_file_picking.json -o picking_order.json -t 'all' -l 5 -v virtual -d baseline -s --task picking" | tee -a $ARC_BASE/output/planner_log_$time.txt

#
# Run picking in virtual with interactive file
#
 script -q -c "python -m pdb planner17.py -j item_location_file.json -o order_file.json -t 'all' -l 5 -v virtual -d baseline -s --task picking --interactive item_location_file.json" | tee -a $ARC_BASE/output/planner_log_$time.txt

#
# Run picking in real
#
#script -q -c "python -m pdb planner17.py -j item_location_file_picking_less_obj.json -o picking_order_less_obj.json -t 'all' -l 5 -d baseline --task picking" | tee -a $ARC_BASE/output/planner_log_$time.txt
#script -q -c "python planner17.py -j item_location_file_picking_more_obj_new_obj.json -o picking_order_more_obj_new_obj.json -t 'all' -l 5 -d baseline --task picking --duration 36000000"  | tee -a $ARC_BASE/output/planner_log_$time.txt
# SS python planner_interactiveplacing17.py -j item_location_file_picking.json -o picking_order.json -t 'all' -l 5 -d baseline --task picking



#
# Run stow as in contest with grasping and flush
#
#script -q -c "python -m pdb planner17.py -j item_location_file.json -t 'all' -l 5  -d baseline " | tee -a $ARC_BASE/output/planner_log_$time.txt


#
# Run stow as in contest with only grasping
#
#script -q -c "rosrun apc_planning planner17.py -j item_location_file.json -t 'all' -l 5  -d baseline -a 'only-grasp-tote' " | tee -a $ARC_BASE/output/planner_log_$time.txt


#
# Run stow as in contest with only grasping
#
#script -q -c "rosrun apc_planning planner17.py -j item_location_file.json -t 'all' -l 5  -d baseline -a 'flush-tote' " | tee -a $ARC_BASE/output/planner_log_$time.txt
#script -q -c "rosrun apc_planning planner17.py -j item_location_file.json -t 'all' -l 5  -d baseline -a 'suction-tote' " | tee -a $ARC_BASE/output/planner_log_$time.txt


#
# Run stow with new object as in contest
#script -q -c "python -m cProfile -o $ARC_BASE/output/planner_prof_$time.txt planner17.py -j item_location_file_with_newobj.json -t 'all' -l 5  -d baseline -a 'suction-tote' --duration 360000" | tee -a $ARC_BASE/output/planner_log_$time.txt

#script -q -c "python -m pdb planner17.py -j item_location_file_with_newobj.json -t 'all' -l 5  -d baseline -a 'suction-tote' --duration 360000" | tee -a $ARC_BASE/output/planner_log_$time.txt
