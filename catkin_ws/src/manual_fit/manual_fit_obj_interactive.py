#!/usr/bin/env python

import roslib; roslib.load_manifest("interactive_markers")
import rospy, tf
from manual_fit.srv import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from pr_msgs.msg import *
from tf.broadcaster import TransformBroadcaster
from ik.roshelper import poselist2pose,poseTransform,rosposeTransform
from interactive_markers.interactive_marker_server import *
global brWorldcam
global brShelf
    
from marker_helper import createMoveControls, createMeshMarker, createCubeMarker
    
import optparse
import math
import copy

def frameCallbackCamera( msg ):
    global brWorldcam
    global currentWorldcamPose
    global listener
    global nworldcam
    global vis_pub
    global opt
    global pyramid_marker
    time = rospy.Time.now()
    
    for i in range(nworldcam):
        p = currentWorldcamPose[i].position
        o = currentWorldcamPose[i].orientation
            
        brWorldcam.sendTransform( (p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
        time, "Worldcam2_%d_rgb_optical_frame" % (i+1), "map" )
    
def frameCallbackShelfTote( msg ):
    global br
    #~ global currentShelfPose
    global currentTotePose
    time = rospy.Time.now()
    
    #~ p = currentShelfPose.position
    #~ o = currentShelfPose.orientation
    
    #~ br.sendTransform( (p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
    #~ time, "shelf", "map")
    
    p = currentTotePose.position
    o = currentTotePose.orientation
    
    br.sendTransform( (p.x, p.y, p.z), (o.x, o.y, o.z, o.w), time, "tote", "map")
    #~ br.sendTransform( (p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
    #~ time, "bin-1", "map")

def processFeedback(feedback):
    global currentExpoPose, currentCrayolaPose, currentWorldcamPose, currentShelfPose, currentJoint_realsense, currentJoint_realsense_link5
    global listener
    import rospy
    p = feedback.pose.position
    o = feedback.pose.orientation
    print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)+", "+str(o.x)+", "+str(o.y)+", "+str(o.z)+ ", "+str(o.w)
    
    if feedback.marker_name == 'ExpoEraser':
        currentExpoPose = feedback.pose
    if feedback.marker_name == 'Crayola':
        currentCrayolaPose = feedback.pose
    if feedback.marker_name == 'Worldcam1':   # todo : improve to nworldcam
        currentWorldcamPose[0] = feedback.pose
    if feedback.marker_name == 'Worldcam2':
        currentWorldcamPose[1] = feedback.pose
    if feedback.marker_name == 'Shelf':
        currentShelfPose = feedback.pose
        p = currentShelfPose.position
        o = currentShelfPose.orientation


def createInteractiveMarker(name, x=0, y=0, z=0, ox=0, oy=0, oz=0, ow=1, frame_id="/map"):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.name = name
    int_marker.scale = 0.3
    int_marker.description = name
    int_marker.pose.position.x = x
    int_marker.pose.position.y = y
    int_marker.pose.position.z = z
    int_marker.pose.orientation.x = ox
    int_marker.pose.orientation.y = oy
    int_marker.pose.orientation.z = oz
    int_marker.pose.orientation.w = ow
    
    global currentExpoPose, currentCrayolaPose
    if int_marker.name == 'ExpoEraser':
        currentExpoPose = int_marker.pose
    if int_marker.name == 'Crayola':
        currentCrayolaPose = int_marker.pose
    
    return int_marker

def getPose(req):
    global currentExpoPose, currentCrayolaPose
        
    print "Returning poses"
    ret = ObjectPoseList()
    
    objpose = ObjectPose()
    objpose.name = 'expo_dry_erase_board_eraser'
    objpose.pose = currentExpoPose
    ret.object_list.append(objpose)
    
    objpose = ObjectPose()
    objpose.name = 'crayola_64_ct'
    objpose.pose = currentCrayolaPose
    ret.object_list.append(objpose)
    
    return GetPoseResponse(ret)

def setShelfPose(req):
    global currentShelfPose
    currentShelfPose = req.pose
    return SetShelfPoseResponse()
    
def setTotePose(req):
    global currentTotePose
    currentTotePose = req.pose
    return SetTotePoseResponse()

#~ def startPoseService():
    #~ s1 = rospy.Service('manualfit_object_pose', GetPose, getPose)
    #~ s2 = rospy.Service('set_shelf_pose', SetShelfPose, setShelfPose)
    #~ s2 = rospy.Service('set_tote_pose', SetTotePose, setTotePose)

def list2pt(pointlist):
    p = Point()
    p.x = pointlist[0]
    p.y = pointlist[1]
    p.z = pointlist[2]
    return p

def createViewMarker(rgba=(1,0.5,0.5,0.5), offset=(0,0,0), orientation=(0,0,0,1), frame_id='map'):
    vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.sleep(0.1)
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = marker.TRIANGLE_LIST
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    
    d = 0.2
    origin = [0,0,0]
    halfThetaU = 0.783984717 /2
    halfThetaV = 1.334023 /2
    points = []
    points.append([math.tan(halfThetaV)*d, math.tan(halfThetaU)*d, d])
    points.append([-math.tan(halfThetaV)*d, math.tan(halfThetaU)*d, d])
    points.append([math.tan(halfThetaV)*d, -math.tan(halfThetaU)*d, d])
    points.append([-math.tan(halfThetaV)*d, -math.tan(halfThetaU)*d, d])
    #print points
        
    color = ColorRGBA()
    marker.points.append(list2pt(origin));    marker.points.append(list2pt(points[0]));    marker.points.append(list2pt(points[1]))
    #marker.colors.append(color)
    
    marker.points.append(list2pt(origin));    marker.points.append(list2pt(points[1]));    marker.points.append(list2pt(points[2]))
    #marker.colors.append(color)
    
    marker.points.append(list2pt(origin));    marker.points.append(list2pt(points[2]));    marker.points.append(list2pt(points[3]))
    #marker.colors.append(color)
    
    marker.points.append(list2pt(origin));    marker.points.append(list2pt(points[3]));    marker.points.append(list2pt(points[0]))
    #marker.colors.append(color)
    
    marker.points.append(list2pt(points[0]));    marker.points.append(list2pt(points[1]));    marker.points.append(list2pt(points[2]))
    #marker.colors.append(color)
    
    marker.points.append(list2pt(points[0]));    marker.points.append(list2pt(points[3]));    marker.points.append(list2pt(points[2]))
    #marker.colors.append(color)
        
    marker.pose.orientation.x = orientation[0]
    marker.pose.orientation.y = orientation[1]
    marker.pose.orientation.z = orientation[2]
    marker.pose.orientation.w = orientation[3]
    marker.pose.position.x = offset[0]
    marker.pose.position.y = offset[1]
    marker.pose.position.z = offset[2]
    
    # for i in range(10):
        # vis_pub.publish(marker)
        # rospy.sleep(0.1)
    
    obj_control = InteractiveMarkerControl()
    obj_control.always_visible = True
    obj_control.markers.append( marker )
        
    return obj_control


def main(argv=None):
    if argv is None:
        argv = sys.argv
    
    parser = optparse.OptionParser()
    
    #import socket
    #useObject = not (socket.gethostname() == 'mcube-002' or socket.gethostname() == 'mcube-003')
    
    parser.add_option('-o', '--obj', action="store_true", dest='useObject', help='To use manual object fitting', 
                      default=False)
                      
    parser.add_option('-c', '--cammarker', action="store_true", dest='useCamMarker', help='To use manual camera fitting or not', 
                      default=False)
                      
    parser.add_option('-s', '--shelfmarker', action="store_true", dest='useShelfMarker', help='To use manual shelf fitting or not', 
                      default=False)
                      
    parser.add_option('-a', '--all', action="store_true", dest='useAllMarker', help='To use manual fitting or not', 
                      default=False)
    
    
    rospy.init_node("manual_fit_server")
    
    global currentExpoPose
    global currentCrayolaPose
    global currentWorldcamPose
    global currentShelfPose
    global currentTotePose
    global currentBin0Pose
    global currentBin1Pose
    global currentBin2Pose
    global currentBin3Pose
    global currentRailsPose
    global currentBinInsideCollisionPose
    global currentBinOutsideCollisionPose
    global br, brWorldcam
    global nworldcam
    global vis_pub
    global opt
    global pyramid_marker
    
    (opt, args) = parser.parse_args()
    currentExpoPose = Pose()
    currentCrayolaPose = Pose()
    currentWorldcamPose = [Pose()]
    currentShelfPose = Pose()
    currentTotePose = Pose()
    currentRailsPose = Pose()
    currentBin0Pose = Pose()
    currentBin1Pose = Pose()
    currentBin2Pose = Pose()
    currentBin3Pose = Pose()
    currentBinInsideCollisionPose = Pose()
    currentBinOutsideCollisionPose = Pose()
    currentJoint_realsense = Pose()
    br = TransformBroadcaster()
    brWorldcam = TransformBroadcaster()
    
    global listener
    listener = tf.TransformListener()
    rospy.sleep(1)
    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("affordance_marker")
    
    # on mcube-002 don't use object
    
    if opt.useObject or opt.useAllMarker:
        # create an interactive marker for Expo
        pose = [1.01179456711, -0.596704602242, -0.126571118832, 0.0, -0.707106769085, 0.0, 0.707106769085]
        int_marker = createInteractiveMarker('ExpoEraser', *pose)
        
        meshmarker = createMeshMarker('package://apc_config/models/object_meshes/expo_dry_erase_board_eraser.stl', 
                                  offset=tuple(pose[0:3]), rgba=(0,1,0,1),
                                  orientation=tuple(pose[3:7]))
        int_marker.controls.append(meshmarker)
        int_marker.controls.extend(createMoveControls())
        server.insert(int_marker, processFeedback)
        
        # create an interactive marker for Crayola
        pose = [1.01179456711, 0.588456273079, 0.0363664329052, 0.0, -0.707106769085, 0.0, 0.707106769085]
        int_marker = createInteractiveMarker('Crayola', *pose)
        meshmarker = createMeshMarker('package://apc_config/models/object_meshes/crayola_64_ct.stl', 
                                  offset=tuple(pose[0:3]), rgba=(1,0,0,1),
                                  orientation=tuple(pose[3:7]))
        int_marker.controls.append(meshmarker)
        int_marker.controls.extend(createMoveControls())
        server.insert(int_marker, processFeedback)
#        
    # storage system
    pose = []
    for term in ['x','y','z','qx','qy','qz','qw']:
        print 'rospy.get_param(tote_pose/ + term)', rospy.get_param('tote_pose/' + term)
        pose.append(rospy.get_param('tote_pose/' + term))
    currentTotePose.position.x = pose[0]
    currentTotePose.position.y = pose[1]
    currentTotePose.position.z = pose[2]
    currentTotePose.orientation.x = pose[3]
    currentTotePose.orientation.y = pose[4]
    currentTotePose.orientation.z = pose[5]
    currentTotePose.orientation.w = pose[6]
    int_marker = createInteractiveMarker('tote', *pose)
    
    meshmarker = createMeshMarker('package://apc_config/models/tote/storage_system_compressed.stl', 
                              offset=tuple(pose[0:3]), rgba=(0.5,0.5,0.5,1), scale = 0.001,
                              orientation=tuple(pose[3:7]), frame_id='/map')
    int_marker.controls.append(meshmarker)
    #~ int_marker.controls.extend(createMoveControls())
    server.insert(int_marker, processFeedback)
    rospy.Timer(rospy.Duration(0.1), frameCallbackShelfTote)
    
    # moving rails
#    pose = []
#    for term in ['x','y','z','qx','qy','qz','qw']:
#        print 'rospy.get_param(tote_pose/ + term)', term))
#rospy.get_param('tote_pose/' + term)
#        pose.append(rospy.get_param('tote_pose/' + 
#    pose[1] = pose[1] + 1*(rospy.get_param('bin3_pose/y') - rospy.get_param('bin2_pose/y'))
#    int_marker = createInteractiveMarker('moving_rails', *pose)
#    
#    meshmarker = createMeshMarker('package://apc_config/models/tote/moving_rails_compressed.stl', 
#                              offset=tuple(pose[0:3]), rgba=(0.5,0.5,0.5,1), scale = 0.001,
#                              orientation=tuple(pose[3:7]), frame_id='/map')
#    int_marker.controls.append(meshmarker)
#    #~ int_marker.controls.extend(createMoveControls())
#    server.insert(int_marker, processFeedback)
#    rospy.Timer(rospy.Duration(0.1), frameCallbackShelfTote)
    # bin0
    pose = []
    for term in ['x','y','z','qx','qy','qz','qw']:
        pose.append(rospy.get_param('bin0_pose/' + term))
    int_marker = createInteractiveMarker('bin0', *pose)
    
    meshmarker = createMeshMarker('package://apc_config/models/bin/bin_large.stl', 
                              offset=tuple(pose[0:3]), rgba=(1,0,0,1), scale = 0.001,
                              orientation=tuple(pose[3:7]), frame_id='/map')
    int_marker.controls.append(meshmarker)
    #int_marker.controls.extend(createMoveControls())
    server.insert(int_marker, processFeedback)
    
    # bin1
    pose = []
    for term in ['x','y','z','qx','qy','qz','qw']:
        pose.append(rospy.get_param('bin1_pose/' + term))
    int_marker = createInteractiveMarker('bin1', *pose)
    
    meshmarker = createMeshMarker('package://apc_config/models/bin/bin_large.stl', 
                              offset=tuple(pose[0:3]), rgba=(1,0,0,1), scale = 0.001,
                              orientation=tuple(pose[3:7]), frame_id='/map')
    int_marker.controls.append(meshmarker)
    #int_marker.controls.extend(createMoveControls())
    server.insert(int_marker, processFeedback)
    # create an interactive marker for left bin (tote) 
    
    # bin2
    pose = []
    for term in ['x','y','z','qx','qy','qz','qw']:
        pose.append(rospy.get_param('bin2_pose/' + term))
    int_marker = createInteractiveMarker('bin2', *pose)
    
    meshmarker = createMeshMarker('package://apc_config/models/bin/bin_large.stl', 
                              offset=tuple(pose[0:3]), rgba=(1,0,0,1), scale = 0.001,
                              orientation=tuple(pose[3:7]), frame_id='/map')
    int_marker.controls.append(meshmarker)
    #int_marker.controls.extend(createMoveControls())
    server.insert(int_marker, processFeedback)
#    
#    # bin3
#    pose = []
#    for term in ['x','y','z','qx','qy','qz','qw']:
#        pose.append(rospy.get_param('bin3_pose/' + term))
#    #~ pose[1] = rospy.get_param('bin2_pose/y')
#    int_marker = createInteractiveMarker('bin3', *pose)
#    
#    meshmarker = createMeshMarker('package://apc_config/models/bin/bin_small.stl', 
#                              offset=tuple(pose[0:3]), rgba=(1,0,0,1), scale = 0.001,
#                              orientation=tuple(pose[3:7]), frame_id='/map')
#    int_marker.controls.append(meshmarker)
#    #int_marker.controls.extend(createMoveControls())
#    server.insert(int_marker, processFeedback)

    ###########
    ## boxes ##
    ###########
    #~boxes 4-5
#    if rospy.get_param('~is_picking', False):
#        pose = []
#        for term in ['x','y','z','qx','qy','qz','qw']:
#            pose.append(rospy.get_param('bin4_pose/' + term))
#        pose[2] = pose[2] - .07512*1
#        int_marker = createInteractiveMarker('box_45', *pose)
#        meshmarker = createMeshMarker('package://apc_config/models/boxes/box_45.stl', 
#                                  offset=tuple(pose[0:3]), rgba=(0.3,0.3,0.3,1), scale = 0.001,
#                                  orientation=tuple(pose[3:7]), frame_id='/map')
#        int_marker.controls.append(meshmarker)
#        server.insert(int_marker, processFeedback)
#        
#        #~box 4
#        pose = []
#        for term in ['x','y','z','qx','qy','qz','qw']:
#            pose.append(rospy.get_param('bin4_pose/' + term))
#        #~ pose[2] = pose[2] + .07512*0
#        int_marker = createInteractiveMarker('cardboard_4', *pose)
#        
#        meshmarker = createMeshMarker('package://apc_config/models/boxes/cardboard_4.stl', 
#                                  offset=tuple(pose[0:3]), rgba=(.75,0.5,0,1.), scale = 0.001,
#                                  orientation=tuple(pose[3:7]), frame_id='/map')
#        int_marker.controls.append(meshmarker)
#        server.insert(int_marker, processFeedback)
#        
#        #~box 5
#        pose = []
#        for term in ['x','y','z','qx','qy','qz','qw']:
#            pose.append(rospy.get_param('bin5_pose/' + term))
#        #~ pose[2] = pose[2] + .1037*1   
#        int_marker = createInteractiveMarker('cardboard_5', *pose)
#        meshmarker = createMeshMarker('package://apc_config/models/boxes/cardboard_5.stl', 
#                                  offset=tuple(pose[0:3]), rgba=(.75,0.5,0,1.), scale = 0.001,
#                                  orientation=tuple(pose[3:7]), frame_id='/map')
#        int_marker.controls.append(meshmarker)
#        server.insert(int_marker, processFeedback)
#    else:

#        
#    #~boxes 6-7
#    pose = []
#    for term in ['x','y','z','qx','qy','qz','qw']:
#        pose.append(rospy.get_param('bin6_pose/' + term))
#    pose[2]-= rospy.get_param('/bin6/z_flap')
#    int_marker = createInteractiveMarker('box_67', *pose)
#    meshmarker = createMeshMarker('package://apc_config/models/boxes/box_67_compressed.stl', 
#                              #~ offset=tuple(pose[0:3]), rgba=(0.1,0.1,0.1,1), scale = 0.001,
#                              offset=tuple(pose[0:3]), rgba=(0.3,0.3,0.3,1), scale = 0.001,
#                              orientation=tuple(pose[3:7]), frame_id='/map')
#    int_marker.controls.append(meshmarker)
#    server.insert(int_marker, processFeedback)
#        
#    #~box 6
#    pose = []
#    for term in ['x','y','z','qx','qy','qz','qw']:
#        pose.append(rospy.get_param('bin6_pose/' + term))
#    #~ pose[2] = pose[2] + 0.07135
#    #~ pose[1] = rospy.get_param('bin2_pose/y')
#    int_marker = createInteractiveMarker('bin6_cardboard', *pose)
#    
#    meshmarker = createMeshMarker('package://apc_config/models/boxes/cardboard_6.stl', 
#                              offset=tuple(pose[0:3]), rgba=(.75,0.5,0,1.), scale = 0.001,
#                              orientation=tuple(pose[3:7]), frame_id='/map')
#    int_marker.controls.append(meshmarker)
#    server.insert(int_marker, processFeedback)
#    
#    #~box 7
#    pose = []
#    for term in ['x','y','z','qx','qy','qz','qw']:
#        pose.append(rospy.get_param('bin7_pose/' + term))
#    #~ pose[1] = rospy.get_param('bin2_pose/y')
#    int_marker = createInteractiveMarker('cardboard_7', *pose)
#    
#    meshmarker = createMeshMarker('package://apc_config/models/boxes/cardboard_7.stl', 
#                              offset=tuple(pose[0:3]), rgba=(.75,0.5,0,1.), scale = 0.001,
#                              orientation=tuple(pose[3:7]), frame_id='/map')
#    int_marker.controls.append(meshmarker)
#    server.insert(int_marker, processFeedback)
#    
#    #~box 8
#    pose = []
#    for term in ['x','y','z','qx','qy','qz','qw']:
#        pose.append(rospy.get_param('bin8_pose/' + term))
#    pose[2]= pose[2] - rospy.get_param('/bin8/z_flap')
#    int_marker = createInteractiveMarker('bin8', *pose)
#    
#    meshmarker = createMeshMarker('package://apc_config/models/boxes/box_8_compressed.stl', 
#                              #~ offset=tuple(pose[0:3]), rgba=(0.5,0.5,0.5,1), scale = 0.001,
#                              offset=tuple(pose[0:3]), rgba=(0.3,0.3,0.3,1), scale = 0.001,
#                              #~ offset=tuple(pose[0:3]), rgba=(0.1,0.1,0.1,1), scale = 0.001,
#                              orientation=tuple(pose[3:7]), frame_id='/map')
#    int_marker.controls.append(meshmarker)
#    server.insert(int_marker, processFeedback)
#    
#    #~box 8
#    pose = []
#    for term in ['x','y','z','qx','qy','qz','qw']:
#        pose.append(rospy.get_param('bin8_pose/' + term))
#    int_marker = createInteractiveMarker('bin8_cardboard', *pose)
#    
#    meshmarker = createMeshMarker('package://apc_config/models/boxes/cardboard_8.stl', 
#                              offset=tuple(pose[0:3]), rgba=(.75,0.5,0,1.), scale = 0.001,
#                              orientation=tuple(pose[3:7]), frame_id='/map')
#    int_marker.controls.append(meshmarker)
#    server.insert(int_marker, processFeedback)
    
    #############
    ## Cameras ##
    #############
    pose = []
    pose.append([1.3216, -.55862, .45089, 0.68986, .678841, -.185574, -.169784]) 
    pose.append([0.54191, -0.54436, 0.28256, 0.65065, -0.63561, 0.29037, -0.29724]) 
    pose.append([1.3225, -0.18962, 0.44935,0.66217, 0.68597, -0.22759, -0.19795]) 
    pose.append([0.48932, -0.18366, 0.038885,-0.42587, 0.42257, -0.56458, 0.56685]) 
    pose.append([0.54073, 0.18461, 0.27804,0.65751, -0.65352, 0.26679, -0.26348]) 
    pose.append([1.3213, -0.55948, 0.37547, 0.58127, 0.60183, -0.40335, -0.37045]) 
    pose.append([1.321, 0.57179, 0.36449,0.60112, 0.58022, -0.38807, -0.38911]) 
    pose.append([0.49027, 0.54894, 0.032927,-0.41015, 0.39011, -0.58548, 0.58035]) 
    pose.append([0.48974, -0.55503, 0.042641,-0.42376, 0.42326, -0.567, 0.5655]) 
    pose.append([1.3201, 0.19125, 0.36687,0.6001, 0.59251, -0.3876, -0.37227]) 
    pose.append([0.54189, -0.17822, 0.28174, 0.6454, -0.64361, 0.28747, -0.29425]) 
    pose.append([1.3247, 0.19321, 0.44441,0.68986, 0.67884, -0.18557, -0.16978]) 
    pose.append([0.5406, 0.55541, 0.27902,0.63946, -0.62909, 0.31036, -0.31467]) 
    pose.append([1.3227, 0.57033, 0.44087,0.68514, 0.67575, -0.1933, -0.19126]) 
    pose.append([1.3206, -0.18846, 0.37101,0.5894, 0.59423, -0.39252, -0.38134]) 
    pose.append([0.48962, 0.17463, 0.035821,-0.41771, 0.40725, -0.57236, 0.57625]) 
    
    #~camera 0
    for i in range(0,16):
        print 'i', i
        pose_tmp = pose[i]
        print 'pose_tmp', pose_tmp
        marker_name = 'realsense_camera'+str(i)
        print 'marker_name', marker_name
        int_marker = createInteractiveMarker(marker_name, *pose_tmp)
        
        meshmarker = createMeshMarker('package://apc_config/models/cameras/realsense_camera_compressed.stl', 
                                  offset=tuple(pose_tmp[0:3]), rgba=(0.1,0.1,0.1,1), scale = 0.001,
                                  orientation=tuple(pose_tmp[3:7]), frame_id='/map')
        int_marker.controls.append(meshmarker)
        server.insert(int_marker, processFeedback)
        
    pose = []
    pose.append([1.3216+.038,0,0.45089+0.015,0,0,0,1])
    pose.append([1.3216+.038,0,0.36449+0.015,0,0,0,1])
    pose.append([0.5406-.038,0,0.27902+0.015,0,0,0,1])
    pose.append([0.48962-.035821,0,0.042641-0.02,0,0,0,1])
    for i in range(0,4):
        pose_temp = pose[i]
        marker_name = 'camera_bar'+str(i)
        int_marker = createInteractiveMarker(marker_name, *pose_temp)
        meshmarker = createMeshMarker('package://apc_config/models/cameras/camera_bar.stl', 
                                  offset=tuple(pose_temp[0:3]), rgba=(0.5,0.5,0.5,1), scale = 0.001,
                                  orientation=tuple(pose_temp[3:7]), frame_id='/map')
        int_marker.controls.append(meshmarker)
        server.insert(int_marker, processFeedback)

    #################
    ## Robot mount ##
    #################
    int_marker = createInteractiveMarker('Robot_mount', 0.02,0,0,0.5,-0.5,-0.5,0.5)
    meshmarker = createMeshMarker('package://apc_config/models/robot_mount/robot_mount_compressed.stl', 
                              offset=(0.02,0,0), rgba=(0.1,0.1,0.1,1),
                              orientation=(0.5,-0.5,-0.5,0.5))
    int_marker.controls.append(meshmarker)
    server.insert(int_marker, processFeedback)

    if opt.useCamMarker or opt.useAllMarker:
        int_marker = createInteractiveMarker('joint_realsense', *pose_world, frame_id='map')

        viewMarker = createViewMarker(frame_id='realsense_depth_optical_frame')

        int_marker.controls.extend(createMoveControls(fixed=True))
        server.insert(int_marker, processFeedback)
        
        vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        pyramid_marker = createMeshMarker('package://apc_config/models/object_meshes/pyramid.stl', 
                      offset=tuple([0,0,0]), rgba=(1,1,0,0.1),
                      orientation=tuple([0,0,0,1]), frame_id='realsense_depth_optical_frame', scale = 0.002, scales = [2, 2,1]).markers[0]
    

    server.applyChanges()
    # ros services
    #startPoseService()
    s1 = rospy.Service('manualfit_object_pose', GetPose, getPose)
    rospy.spin()
    

if __name__=="__main__":
    sys.exit(main())

