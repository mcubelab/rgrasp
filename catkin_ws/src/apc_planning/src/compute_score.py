#!/usr/bin/env python
# suction down primitive:

# inputs:
# Pose and position of the object, object ID, shelf pose, position and force
# threshold for grasp.

# fails:
# # The vertical dimension of the object should be smaller than the maximum 
# gap distance between the two fingers.

import tf
from ik.ik import IK, IKJoint, Plan, setSpeedByName
import rospy
import numpy as np
import tf.transformations as tfm
import goToHome
import percept
import copy
from ik.helper import getBinMouth, get_bin_cnstr, find_object_pose_type, bin_to_world, world_to_bin, matrix_from_xyzquat
from ik.roshelper import coordinateFrameTransform, pubFrame, pose2list, ROS_Wait_For_Msg


def VisualizeHeightMap(H):
    [range_x,range_y] = H.shape
    for x in xrange(int(0.75*range_x)-1,-1,-1):
        st = str(x%10)
        for y in xrange(range_y-1,-1,-1):
            if H[x,y]>0:# or G[x,y]>0:
#                if G[x,y]!=H[x,y]:
#                    if H[x,y]==0:
#                        st = st +  '\033[92m' + '#' + '\033[0m'
#                    elif H[x,y]>G[x,y]:
#                        st = st +  '\033[92m' + 'L' + '\033[0m'
#                    else:
#                        st = st +  '\033[92m' + 'T' + '\033[0m'
#                else:
                    st = st + '\033[94m' + '#' + '\033[0m'
            else:
                st = st +  '.'
        print st
    st = ' '
    for y in range(range_y-1,-1,-1):
        st = st + str(y%10)
    print st

def VisualizeScores(H,Scores,max_score):
    GREEN = '\033[92m'
    WHITE = '\033[0m'
    ORANGE = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    orange_threshold = 0.75
    [range_x,range_y] = H.shape
    for x in xrange(int(0.6*range_x)-1,-1,-1):
        st = WHITE + str(x%10)
        for y in xrange(range_y-1,-1,-1):
            if Scores[x][y]==max_score: #Red
                st = st +  RED
            elif Scores[x][y]>max_score*orange_threshold: #Orange
                st = st + ORANGE
            elif Scores[x][y] < 0:
                if H[x,y] > 0:
                    st+= BLUE
                else:
                    st+=WHITE
            else:
                st += GREEN
            if H[x,y]>0:# or G[x,y]>0:
#                if G[x,y]!=H[x,y]:
#                    if H[x,y]==0:
#                        st = st +  '\033[92m' + '#' + '\033[0m'
#                    elif H[x,y]>G[x,y]:
#                        st = st +  '\033[92m' + 'L' + '\033[0m'
#                    else:
#                        st = st +  '\033[92m' + 'T' + '\033[0m'
#                else:
                st +='#'
            else:
                if Scores[x][y] <0: #Normal points
                    st = st +  '.'
                else:
                    st+= '*'
        print st
    st = ' '
    for y in range(range_y-1,-1,-1):
        st = st + str(y%10)
    print st

def get_grid(listener,
             br,
             binNum=4,
             robotConfig = None,
             shelfPosition = [1.9019,0.00030975,-0.503],
             obstacles_id = None,
             isExecute = True,
             withPause = True):
    ###Given the bin and the objects into it, we compute a grid for the bin heights
    print '[Get_grid of bin %d] start!' % binNum 
 
    #GET OBSTACLE INFO
    goal_object, obj_pose_type, list_pose_objects=percept.percept_AprilTag(obj_id = obstacles_id[0], binNum = binNums[0],bin_contents = obstacles_id)
    
    # Poses and Id of the obstacles in the bin
    print 'list_pose_objects: ', list_pose_objects
    obstacles_poses = [ world_to_bin(list_pose_objects[obj], binNums[0])for obj in list_pose_objects]
    obstacles_id = [ obj for obj in list_pose_objects]
    print 'obstacles_poses: ',obstacles_poses
    obstacles_poses = np.array(obstacles_poses)
    obstacles = obstacles_poses  
    
    joint_topic = '/joint_states'
        
    #Grid parameters and bin variables
    distFromShelf = 0.05
    (binMouth,bin_height,bin_width) = getBinMouth(distFromShelf, binNum)
    print 'bin_width: ', bin_width
    bin_depth = 0.47
    #Grid parameters for the XY plane of the shelf; each element of the grid is 0.5*0.5 cm
    range_x = 2*int(100*bin_depth)
    range_y = 2*int(100*bin_width)
    H_grid = np.zeros((range_x, range_y))
    
    print 'Obstacles: ' , obstacles
    #fill H_grid with the height in each square. For each object and grid square check the heigh occupied with the funcion 'in_or_out'.
    for i in range(len(obstacles_id)):
        object_size = get_obj_dim(obstacles_id[i])
        for x in range(range_x):      #we should use a corner, as we did with the object, aixo nomes n'agafa 4, no 5
            for y in range(range_y):
                h_obst = in_or_out(grid_x = x,grid_y = y,
                m_per_x = bin_depth/range_x, m_per_y = bin_width/range_y, 
                position = obstacles[i,0:3], 
                orientation = obstacles[i,3:7],
                dim = object_size)
                H_grid[x,y] = max(H_grid[x,y], h_obst)
    return H_grid
                
    
def get_score(H_grid,
            objPose_in_hand = [1.95,1.25,1.4,0,0,0,1],
            objId = 0,
            binNum=4,
            robotConfig = None, 
            isExecute = True,
            grasping = True,
            withPause = True):
    ##Find position with the best score!  
    goal_position = [-1,-1, -1]
    threshold = -5
    best_score = threshold
    
    #------------Object dimensions------------------
    object_size = np.array(get_obj_dim(objId))
    obj_pose_tfm=np.array(matrix_from_xyzquat(objPose_in_hand[0:3], objPose_in_hand[3:7]))
    obj_pose_orient=obj_pose_tfm[0:3,0:3]
    
    x_index=np.argmax(np.fabs(obj_pose_orient[0,0:3]))
    x_dim=object_size[x_index]
    y_index=np.argmax(np.fabs(obj_pose_orient[1,0:3]))
    y_dim=object_size[y_index]
    z_index=np.argmax(np.fabs(obj_pose_orient[2,0:3]))
    z_dim=object_size[z_index]
    print 'x_dim: ', x_dim
    print 'y_dim: ', y_dim
    print 'z_dim: ', z_dim
    #------------------------------------------------
    
    #-----------------Parameters---------------------
    bottomlipHeight = 0.02
    lipHeight = 0.035
    
    #TODO: look why 2 different finger_width
    finger_width =  0.03# Now we include the spatula, before 0.02
    finger_height = 0.062 #Lowdate for the new ones: 0.060
    finger_length = 0.18 # TODO: update when they work 0.215 with spatulas
    temporal_width_hack = 0 #TODO: now the gripper works :)
    
    cup_length = 0.027 
    x_offset = 0.01  #TODO: x_offset could be calculated with respect to the object position in the gripper vs. the lenght of the fingers
    
    ## We add the dimensions of the gripper to the object to ensure that it will fit into the goal_pose
    #------------------------------------------------
    distFromShelf = 0.05
    (binMouth,bin_height,bin_width) = getBinMouth(distFromShelf, binNum)
    print 'bin_width: ', bin_width
    bin_depth = 0.47
    range_x = 2*int(100*bin_depth)
    range_y = 2*int(100*bin_width)
    
    if grasping:
        #TODO: this only contemplates the case where the gripper is horizontal, we may want to introduce the case where the griper is vertical...
        object_size = np.array([max(x_dim+x_offset,0), max(temporal_width_hack, y_dim+2*finger_width), max(z_dim, finger_height) + lipHeight])
    else:
        object_size = np.array([max(x_dim+x_offset,0), max(y_dim+finger_width, 2*finger_width), z_dim+cup_length+lipHeight+finger_height])  #TODO: if both suctions are working y_dim instead of y_dim+finger_width
        #TODO: the z dimensions we are adding z_dim to finger_heigh which is probably more like a maximum: max(z_dim, cup_length + finger_height)
        #TODO: y_dim+finger_width is a conservative guess
    
    ##TODO: orient following quaternion
    ##ASUMPTION: the object is a 3D rectangle
    #Number squares the object occupies
    squares_x = int(object_size[0]*range_x/bin_depth+0.99)
    squares_y = int(object_size[1]*range_y/bin_width+0.99)
    
    height_obj = object_size[2]
    Scores = -10000*np.ones([range_x,range_y])
    print 'Size object: ', squares_x, squares_y
    #We try the places where it can fit, looking at its corner
    for x in range(range_x-squares_x+1):
        for y in range(range_y-squares_y+1):
            score = 0
            ####Calculate score for each option####
            suma = 0
            sum_sq = 0
            maximum = 0
            minimum = 0
            #Heigh in the position for the object
            for dx in range(squares_x):
                for dy in range(squares_y):
                    nx = x+dx
                    ny = y+dy
                    height = H_grid[nx,ny]
                    suma = suma + height
                    sum_sq = sum_sq + height*height
                    maximum = max(maximum,height)
                    minimum = min(minimum,height)
            #Heigh in the positions in front of the object
            max_front = 0
            for front_x in range(x):
                for dy in range(squares_y):
                    max_front = max(max_front,H_grid[front_x,y+dy])
            
            suma = suma/(squares_x*squares_y)
            sum_sq = sum_sq / (squares_x*squares_y) #useful for variance
            height_std = math.sqrt(sum_sq-suma*suma)
            ###########################
            ### Calculating score #####
            ###########################
            
            #Penalty for going in
            score = 1 - 1.5*x*bin_depth/range_x 
            #if float(x)<0.015*range_x/0.47: #Penalty for close to begining
            #    score = score - 1000
            #Penalty for objects in front
            if max_front > 0.10:
                score -=1000
            elif max_front > 0:
                score -= 1*max_front
            else:
                score += 0.05
            #Penalty for objects on top of objects
            if maximum > 0.10:
                score -= 1000
            elif maximum > 0:
                score -= max(0,maximum-max_front)
            else:
                score += 0.05
            #Penalty for uneven surfaces
            if height_std > 0.075:
                score -= 1000
            elif height_std==0:
                score+=0.01
            else:
                score-=height_std
            #Penalty for being on the sides
            if float(y+squares_y-1)> 0.9*float(range_y) or float(y) < 0.1*float(range_y):
                score-=10*max(float(y+squares_y-1)-0.9,0.1-float(y))
                
            ###########################
            ### End calculation score##
            ###########################
            
            Scores[x+int(squares_x/2),y+int(squares_y/2)] = score
            if score >best_score:
                best_score = score
                goal_position = np.array([x,y, max(maximum,max_front)])
    #np.set_printoptions(threshold='nan')
    print 'Scores: ', Scores
    print 'best_score: ',best_score
    #print 'H_grid: ', H_grid
    print 'goal_position: ', goal_position
    #VisualizeHeightMap(H_grid)
    
    VisualizeScores(H_grid,Scores,best_score)
    if best_score <= threshold:
        print '[Placing] Failed to find a good spot'
        #TODO: check if that's the proper way to do it.
        return False, best_score, goal_position
    #TODO: there is 1 centimiter added at the z coordinate
    pos_obj_goal = np.array([bin_depth/range_x*goal_position[0]+object_size[0]/2.,bin_width/range_y*goal_position[1]+object_size[1]/2.,goal_position[2]+height_obj/2+0.01])
    return True, best_score, goal_position, pos_obj_goal
    
def placing(listener,
            br,
            objPose_in_hand = [1.95,1.25,1.4,0,0,0,1],
            binNum=4,
            objId = 0,
            goal_position = 0,
            pos_obj_goal = [1.95,1.25,1.4,0,0,0,1],
            bin_contents = None,
            robotConfig = None, 
            shelfPosition = [1.9019,0.00030975,-0.503], 
            obstacles_id = None, ##16
            forceThreshold = 1, 
            isExecute = True,
            withPause = True,
            grasping = True):
    ## objPose: world position and orientation frame attached to com of object in quaternion form. XYZ before the object is placed
    ## objId: object identifier
    ## robotConfig: current time robot configuration
    ## shelfPosition: shelf position in world frame
    ## force threshold: amount fo force needed to say we have a grasp
    ## obstacles and obstacles_id: position and identifier of the object in the bin
    print '[Placing] start!'
    
    
    # shelf variables
    bottomlipHeight = 0.02
    lipHeight = 0.035
    
    
    joint_topic = '/joint_states'
    #Grid parameters and bin variables
    distFromShelf = 0.05
    (binMouth,bin_height,bin_width) = getBinMouth(distFromShelf, binNum)
    print 'bin_width: ', bin_width
    
    #TODO: look why 2 different finger_width
    finger_width =  0.03# Now we include the spatula, before 0.02
    finger_height = 0.062 #Lowdate for the new ones: 0.060
    finger_length = 0.18 # TODO: update when they work 0.215 with spatulas
    temporal_width_hack = 0 #TODO: now the gripper works :)
    
    cup_length = 0.027 
    x_offset = 0.01  #TODO: x_offset could be calculated with respect to the object position in the gripper vs. the lenght of the fingers
    
    ## We add 
    
    #Ara ben fet:
    augmented_pos_obj_goal = [pos_obj_goal[0], pos_obj_goal[1], pos_obj_goal[2], 0, 0,0,1]
    objPose = bin_to_world(augmented_pos_obj_goal,binNum)[0:3]
    
    print 'objPose: ', objPose
    ##########given all the object position in the world, we need to add orientation
    ## get object position (world frame)  --> and vertical height
    objPosition = getObjCOM(objPose[0:3], objId)
    obj_pose_tfm_list=matrix_from_xyzquat(objPose[0:3], objPose_in_hand[3:7])
    obj_pose_tfm=np.array(obj_pose_tfm_list)
    
    obj_pose_orient=obj_pose_tfm[0:3,0:3]
    vertical_index=np.argmax(np.fabs(obj_pose_orient[2,0:3]))
    
    object_dim=get_obj_dim(objId)
    object_dim=adjust_obj_dim(objId,object_dim)
    vertical_dim=object_dim[vertical_index]

	#---------------
	#actual positions, this could be MOVED
    while True:
        APCrobotjoints = ROS_Wait_For_Msg(joint_topic, sensor_msgs.msg.JointState).getmsg() 
        q0 = APCrobotjoints.position
        if len(q0) >= 6:
            q0 = q0[0:6]   # take first 6, because in virtual environmet there will be additional 2 hand joint
            break
    #---------------
        
    ## move gripper to object com outside bin along world y direction and
    ## move the gripper over the lip of the bin    
    
    # set tcp
    vert_offset= 0.0 #hand_fing_disp_Xoffset #PROBABLY CHANGE
    grasp_l2=0.0
    grasp_l3 = 0.33#+0.07 when spatulas work
    tip_hand_transform = [vert_offset, grasp_l2, grasp_l3, 0,0,0] # to be updated when we have a hand design finalized
      
    #Old values
    #vert_offset=.035
    #l2 =.44
    #tip_hand_transform = [-vert_offset, 0, l2, 0,0,0] # to be updated when we have a hand design finalized
    
    # broadcast frame attached to tcp
    pubFrame(br, pose=tip_hand_transform, frame_id='tip', parent_frame_id='link_6', npub=5)
    
    # get position of the tcp in world frame
    pose_world = coordinateFrameTransform(tip_hand_transform[0:3], 'link_6', 'map', listener)
    tcpPos=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
       
    # set scoop orientation (rotate wrist) --> used in generate_plan
    scoopOrientation = [0.5,0.5,0.5,0.5]#[0.7071, 0, 0.7071,0]#[0, 0.7071,0, 0.7071]
 
    #bin position     
    distFromShelf = 0.05
    wristWidth = 0.0725 # this is actually half the wrist width
    #(binMouth,bin_height,bin_width) = getBinMouth(distFromShelf, binNum)
    pose_world = coordinateFrameTransform(binMouth[0:3], 'shelf', 'map', listener)
    print 'binMouth: ', binMouth
    binMouth=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    print 'binMouth in map: ', binMouth
    #close hand
    hand_gap=0
    #gripper.close()
    
    #more parameter
    #finger_length=.215 #195     
    #finger_width=.08
    
    up_offset=.05
    down_offset=-.01  #.025
    cup_to_spatula=.095
    hand_width=.15
    max_gripper_width=110
    hand_top_offset=hand_width/2+vert_offset+.01
    hand_bot_offset=hand_width/2-vert_offset
    
    #this determines bin stuff based on bin input
    binFloorHeight=binMouth[2]-bin_height/2
    binCeilHeight=binMouth[2]+bin_height/2
    
    #this determines bin sides based on object
    (binSmall,binLarge)=getSides(binNum,listener)
    horz_offset=0    
    small_limit=binSmall+finger_width/2+horz_offset
    large_limit=binLarge-finger_width/2-horz_offset
    
    plans = []
    plan = Plan()
    #start_config=possible_start_config
    start_config=[q0[0],q0[1],q0[2],q0[3],q0[4], q0[5]];
    planner = IKJoint(q0 = q0, target_joint_pos=start_config)
    plan = planner.plan()   
    plans.append(plan) 
    qf=plan.q_traj[-1]  #whats that?
    
    
    def get_motion_param(target_x,target_y):
        object_depth=target_x-binMouth[0]
        sidepos=min(max(target_y,small_limit),large_limit)
        
        if object_depth<finger_length:
            print '[Placing] shallow suction'
            h1=binCeilHeight- lipHeight - (cup_length+finger_height)/2.#-cup_to_spatula #in this case cup_to_spatula should be the finger height + cup
            h2=binFloorHeight+vertical_dim + goal_position[2] #-down_offset
        else:
            print '[Placing] deep suction, quitting'
            h1=binCeilHeight-lipHeight-hand_top_offset
            h2a=binFloorHeight+vertical_dim-down_offset
            h2b=binFloorHeight+hand_bot_offset+lipHeight
            h2=max(h2a,h2b)
            return False, h1,h2,sidepos

        h2=max(h2,binFloorHeight)
                
        print 'h2:', h2
        print 'h1:', h1
           
            
        if h2>h1:
            print '[Placing] cant go in'
            return False, h1,h2,sidepos
        return True,h1,h2,sidepos

    def generate_plan(targetPositionList,plans,qf):
        for tp_index in range(0, len(targetPositionList)):
            
            targetPosition = targetPositionList[tp_index]
            planner = IK(q0 = qf, target_tip_pos = targetPosition, target_tip_ori = scoopOrientation,
            tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
            
            plan = planner.plan()
            s = plan.success()
            print '[Placing] Plan number:',tp_index+1
            if s:
                print '[Placing] Plan calculated successfully'
                plans.append(plan)

            else:
                print '[Placing] Plan calulation failed'
                return (False, plans,qf)
                
            qf = plan.q_traj[-1]
        return (True,plans,qf)
    
    def execute_forward(plans,hand_gap):
        for numOfPlan in range(0, len(plans)):
            if isExecute:
                plans[numOfPlan].visualize(hand_param=hand_gap)
                pauseFunc(withPause)
                plans[numOfPlan].execute()
    
    def execute_backward(plans,hand_gap,plan_offset):
        for numOfPlan in range(0, len(plans)-plan_offset):
            plans[len(plans)-numOfPlan-1].visualizeBackward(hand_param=hand_gap)
            if isExecute:
                pauseFunc(withPause)
                plans[len(plans)-numOfPlan-1].executeBackward()
  
  
	#position where we want to place our object
    target_offset=get_test_offset(objId)
    target_x_list=[objPosition[0]] #,objPosition[0]-target_offset,objPosition[0],objPosition[0],objPosition[0]+target_offset]
    target_y_list=[objPosition[1]] #,objPosition[1],objPosition[1]+target_offset,objPosition[1]-target_offset,objPosition[1]]
    print objPosition
    count=0
    
    suction_succeed=False
    
    overall_plan_succeed=False
    
    #see if it can bee done
    (continue_val,h1,h2,sidepos)=get_motion_param(target_x=target_x_list[0],target_y=target_y_list[0])    
        
    if continue_val==False:
        return False,False
    
    #close to bin    
    targetPositionListA=[
    [binMouth[0]-.15, sidepos, h1]]
    #,
    #[binMouth[0]-.15, sidepos, h1]]
    
    
    #in the bin
    targetPositionListB=[
    [target_x_list[0], sidepos, h1]]
    
    #TODO: go down until contact
    h1 = min(h1, binFloorHeight+max(vertical_dim/2, (cup_length+finger_height)/2) + goal_position[2])
    targetPositionListC=[
    [target_x_list[0], sidepos, h1]]
    
    
    
    
    (continue_val,plans1,qf)=generate_plan(targetPositionListA,plans,qf)
    
    if continue_val==False:
        return False,False
        
    (continue_val,plans2,qf)=generate_plan(targetPositionListB,[],qf)
    
    if continue_val==False:
        return False,False
    
    (continue_val,plans3,qf)=generate_plan(targetPositionListC,[],qf)
    
    if continue_val==False:
        return False,False
        
    setSpeedByName(speedName = 'slow')
    execute_forward(plans1,0)
    
    setSpeedByName(speedName = 'slow')
    execute_forward(plans2,0)
    
    setSpeedByName(speedName = 'slow')
    execute_forward(plans3,0)
    #TODO: Addapt to object in hand; open sufficiently so that the object goes down, but not too much
    
    if grasping == True:
        moveGripper(y_dim*1.02,20) #TODO: open has much as the object
        #TODO: you would like to close if the object is going to go down for saftiness
        spatula.sp1_setPosition(0)
        spatula.sp2_setPosition(0)
    else: 
        suction.stop()
    time.sleep(0.6)
    
    #go out the bin
    execute_backward(plans3,0,0)
    execute_backward(plans2,0,0)
    
    overall_plan_scceed = True;
    suction_succeed = True;
    return (overall_plan_succeed,suction_succeed)


  


def get_test_offset(objId):
    if objId=='champion_copper_plus_spark_plug':
        return .01
    if objId=='mead_index_cards':
        return .025
    if objId=='kong_air_dog_squeakair_tennis_ball':
        return .053
    if objId=='mommys_helper_outlet_plugs':
        return .05
    return .015
    
def adjust_obj_dim(objId,object_dim):
    if objId=='champion_copper_plus_spark_plug':
        object_dim[0]=object_dim[1]
    if objId=='kong_air_dog_squeakair_tennis_ball':
        object_dim[0]=.03
        object_dim[1]=.03
        object_dim[2]=.03
        
    return object_dim
    


    
def getSides(binNum,listener):
             
    bin_cnstr = get_bin_cnstr()
                 
    AdjustedX_list=[bin_cnstr[2][0]+.045,bin_cnstr[2][1],bin_cnstr[1][1],bin_cnstr[0][1]-.045]
    print '[Placing] AdjustedX_list:', AdjustedX_list
    
    i=2-(binNum%3)
    shelf_gap=AdjustedX_list[i+1]-AdjustedX_list[i]
    shelfXVec=[AdjustedX_list[i+1],0,0]
            
    newShelfXVec=coordinateFrameTransform(shelfXVec, 'shelf', 'map', listener)
            
    return (newShelfXVec.pose.position.y-shelf_gap,newShelfXVec.pose.position.y)
    




if __name__=='__main__':
    
    rospy.init_node('listener', anonymous=True)
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    shelf_world_transform = coordinateFrameTransform([0, 0, 0, 0, 0, 0, 1], 'shelf', 'map', listener)
    
    #Bin where we compute the height
    binNums = [4]
    
    #Object pose in hand
    objPoses = []
    objPoses.append([1.599, 0.450, 1.106, 0.7071,0 ,0, 0.7071]) #Only orientation matters
    
    
    #Obstacles in that bin 
    obstacles_id = [ 'dove_beauty_bar','laugh_out_loud_joke_book']#'dove_beauty_bar', 'kleenex_tissue_box', 'laugh_out_loud_joke_book', 'expo_dry_erase_board_eraser'] 
    
    #Execute the placing of the object and then go home
    H_grid = get_grid(
    listener = listener,
    br = br,
    binNum = binNums[0],
    robotConfig=None, 
    shelfPosition = (pose2list(shelf_world_transform.pose))[0:3],
    obstacles_id = obstacles_id,
    isExecute = True,
    withPause = True)
    goToHome.goToHome()  
    
    #Execute the placing of the object and then go home
    [is_possible, best_score, goal_position, pos_obj_goal]= get_score(
    objPose_in_hand = objPoses[0],
    objId='crayola_24_ct', #'elmers_washable_no_run_school_glue',#'expo_dry_erase_board_eraser' ,
    grasping = True,
    H_grid = H_grid,
    isExecute = True,
    withPause = True)
    
    placing(
    listener=listener,
    br=br,
    objPose_in_hand = objPoses[0],
    goal_position = goal_position,
    pos_obj_goal = pos_obj_goal,
    binNum = binNums[0],
    objId='crayola_24_ct', #'elmers_washable_no_run_school_glue',#'expo_dry_erase_board_eraser' ,
    robotConfig=None, 
    shelfPosition = (pose2list(shelf_world_transform.pose))[0:3], #[1.9019,0.00030975,-0.503], 
    obstacles_id = obstacles_id,
    isExecute = True,
    withPause = True,
    grasping = True)
    goToHome.goToHome()
                  

