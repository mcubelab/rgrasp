

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


#
# Run test in virtual: it will run the stowing task with 20 items for both primitives: suction and grasping. Randomly primitives will fail.
#
python planner17.py -j item_location_file_set_3_new_obj.json -t 'all' -l 5 -v virtual -d random -s
#python planner17.py -j item_location_file.json -t 'all' -l 5 -v virtual -d random -s


#
# Run test in virtual: it will run the stowing task with 20 items for suction.
#
#~ python planner17.py -j item_location_file.json -t 'all' -l 5 -v virtual -d random -s -a 'suction-tote'

#
# Run test in virtual: it will run the stowing task with 20 items for grasping.
#
#python planner17.py -j item_location_file.json -t 'all' -a 'grasp-tote' -l 5 -v virtual -d random -s

# Run test in virtual: it will run the stowing task with 20 items for flush gaspring.
#
#python planner17.py -j item_location_file.json -t 'all' -a 'flush-tote' -l 5 -v virtual -d random -s

