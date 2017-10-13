#!/bin/bash

rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '612203002922'" > /dev/null
rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '614203000465'" > /dev/null
rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '612203004574'" > /dev/null
rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '616205005772'" > /dev/null
rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '616205001219'" > /dev/null
rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '61420501085'" > /dev/null
rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '616205004776'" > /dev/null
rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '614203003651'" > /dev/null
rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '612205002211'" > /dev/null
rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '612204001396'" > /dev/null
rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '614205001856'" > /dev/null
rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '613201001839'" > /dev/null
rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '617205001931'" > /dev/null
rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '612201002220'" > /dev/null
rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '616203002024'" > /dev/null
rosservice call /arc_1/realsense_camera/capture "camera_serial_number: '614204001012'" > /dev/null

#~ cam_configs = {'612203002922': {'bin_num': 'bin0', 'place': 'passive_near', 'ns': 'arc_1'}, 
              #~ '614203000465': {'bin_num': 'bin0', 'place': 'passive_far', 'ns': 'arc_1'}, 
              #~ '612203004574': {'bin_num': 'bin0', 'place': 'active_near', 'ns': 'arc_1'}, 
              #~ '616205005772': {'bin_num': 'bin0', 'place': 'active_far', 'ns': 'arc_1'},

              #~ '616205001219': {'bin_num': 'bin1', 'place': 'passive_near', 'ns': 'arc_1'}, 
              #~ '61420501085': {'bin_num': 'bin1', 'place': 'passive_far', 'ns': 'arc_1'}, 
              #~ '616205004776': {'bin_num': 'bin1', 'place': 'active_near', 'ns': 'arc_1'}, 
              #~ '614203003651': {'bin_num': 'bin1', 'place': 'active_far', 'ns': 'arc_1'},

              #~ '612205002211': {'bin_num': 'bin2', 'place': 'passive_near', 'ns': 'arc_1'}, 
              #~ '612204001396': {'bin_num': 'bin2', 'place': 'passive_far', 'ns': 'arc_1'}, 
              #~ '614205001856': {'bin_num': 'bin2', 'place': 'active_near', 'ns': 'arc_1'}, 
              #~ '613201001839': {'bin_num': 'bin2', 'place': 'active_far', 'ns': 'arc_1'},

              #~ '617205001931': {'bin_num': 'bin3', 'place': 'passive_near', 'ns': 'arc_1'}, 
              #~ '612201002220': {'bin_num': 'bin3', 'place': 'passive_far', 'ns': 'arc_1'}, 
              #~ '616203002024': {'bin_num': 'bin3', 'place': 'active_near', 'ns': 'arc_1'}, 
              #~ '614204001012': {'bin_num': 'bin3', 'place': 'active_far', 'ns': 'arc_1'}}
