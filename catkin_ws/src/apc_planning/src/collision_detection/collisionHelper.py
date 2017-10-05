#~ from ik.ik import *
import sys
import os
rgrasp_path = os.environ['CODE_BASE'] + '/catkin_ws/src/apc_planning/src'
sys.path.append(rgrasp_path)
import rospy
import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pylab as plt
from suction_projection import suction_projection_func
import goToHome 
import ik.helper
#from ik.helper import matrix_from_xyzquat, reference_frames, get_params_yaml
import ik.ik

def getToteSections(listener, br, flag = True):
        #~build outside sections and floor
        world_X, world_Y, world_Z, tote_X,tote_Y,tote_Z, tote_pose_pos = ik.helper.reference_frames(listener = listener, br=br)
        zheight=rospy.get_param("/tote/height")
        bodies = []
        for i in range(0,4):
            bodies.append(getBinPoints(i, listener=listener, br=br))
            #~ print 'body[i]', bodies[i]
        bodies.append(buildCenteredBox(length=1.5, width=3, heigth=zheight, midPos=tote_pose_pos, listener=listener, br=br))
        #~ print 'body[i]', bodies[4]
        #~build sections from bodies
        section_body_list = []
        section_body_list.append([4,3])
        section_body_list.append([4,3,0])
        section_body_list.append([4,0])
        section_body_list.append([4,3,0])
        section_body_list.append([0,1])
        section_body_list.append([1,2])
        section_body_list.append([2,3])
        #~build sections from points
        section_point_list = []
        section_point_list.append([[0,1,2,3],[0,1,2,3]])
        section_point_list.append([[2,3,4,5],[2,3],[4,5]])
        section_point_list.append([[4,5,6,7],[4,5,6,7]])
        section_point_list.append([[0,1,6,7],[0,1],[6,7]])
        section_point_list.append([[0,1,2,3],[4,5,6,7]])
        section_point_list.append([[0,1,2,3],[4,5,6,7]])
        section_point_list.append([[0,1,2,3],[4,5,6,7]])
        #~build sections
        section_tmp = []
        sectionList = []
    
        body_counter = 0
        section_list_tmp = [1]
        for section in range(0,len(section_body_list)):
            for body_counter in range(0,len(section_body_list[section])):
                for point in section_point_list[section][body_counter]:
                    body = section_body_list[section][body_counter]
                    section_tmp.append(bodies[body][point])
            sectionList.append(np.array(section_tmp))
            section_tmp = []

        return sectionList
        
def getBinPoints(binId, listener, br, isSuction=False):
    #~build box collision geometries
    bodies = []
    tol = 0.04
    xlength = rospy.get_param('/bin' + str(binId) + '/length')
    ywidth = rospy.get_param('/bin' + str(binId) + '/width')
    zheight = rospy.get_param('/bin' + str(binId) + '/height')
    #~build box
    bin_center_pose = ik.helper.get_params_yaml('bin'+str(binId)+'_pose')
    bodies.append(buildCenteredBox(length=xlength-tol*2, width=ywidth-tol*2, heigth=zheight, midPos=bin_center_pose[0:3], listener=listener, br=br))

    return np.asarray(bodies[0])
    

def buildCenteredBox(length, width, heigth, midPos, listener, br):
    world_X, world_Y, world_Z, tote_X,tote_Y,tote_Z, tote_pose_pos = ik.helper.reference_frames(listener=listener, br=br)
    #Build center points
    #
    TotePoints = []
    a = [[1,1,0],[1,1,-1],[-1,1,0],[-1,1,-1],[-1,-1,0],[-1,-1,-1],[1,-1,0],[1,-1,-1]] #~[X,Y,Z]

    for i in range(0, 8):
        TotePoints.append(midPos +a[i][0]*(length/2.0)*tote_X+a[i][1]*(width/2.0)*tote_Y + a[i][2]*(heigth)*tote_Z)

    return np.asarray(TotePoints)

def getFingerPoints(finger_opening, tcp_pos, hand_orient_norm, isSuction):
        #Get finger dimensions from yaml file
        finger_width = rospy.get_param("/finger/width")
        dist_tcp_to_intersection = rospy.get_param("/wrist/length")
        dist_tcp_to_spatula = rospy.get_param("/gripper/spatula_tip_to_tcp_dist")
        finger_thickness_top = rospy.get_param("/finger/thickness_top")
        finger_thickness_bot = rospy.get_param("/finger/thickness_bot") + 0.015 #~add distance for spatula
        #Get hand reference frame
        hand_X=hand_orient_norm[0:3,0]
        hand_Y=hand_orient_norm[0:3,1]
        hand_Z=hand_orient_norm[0:3,2]
        #offset of fingers
        fing_offset = 5.0/1000.0
        #~build finger body
        FingerBody = []
        b = [(1,1),(1,-1),(-1,1),(-1,-1)]
        for i in range(0, 4):   
            #~Finger Body (changes with opening)
            if b[i][0]==1:
                finger_thickness = finger_thickness_top
            else:
                finger_thickness = finger_thickness_bot
            FingerBody.append(tcp_pos+dist_tcp_to_intersection*hand_Z + b[i][0]*(finger_opening/2.0+finger_thickness)*hand_X+ ((b[i][1]*(finger_width/2.0)+fing_offset))*hand_Y)
            FingerBody.append(tcp_pos+dist_tcp_to_spatula*hand_Z + b[i][0]*(finger_opening/2.0+finger_thickness)*hand_X+(b[i][1]*(finger_width/2.0)+fing_offset)*hand_Y)

        return np.asarray(FingerBody)

def getWristPoints(tcp_pos, hand_orient_norm, isSuction):
        #Get finger dimensions from yaml file
        dist_tcp_to_intersection = rospy.get_param("/wrist/length")
        gripper_width_top = rospy.get_param("/scorpion_tail/grasp/top_grasp_width")#~to change
        gripper_width_bot = rospy.get_param("/scorpion_tail/grasp/bot_grasp_width")#~to change
        gripper_length = rospy.get_param("/scorpion_tail/width")
        #Get hand reference frame
        hand_X=hand_orient_norm[0:3,0]
        hand_Y=hand_orient_norm[0:3,1]
        hand_Z=hand_orient_norm[0:3,2]
        #offset of fingers
        fing_offset = (5.0/1000.0)*1
        #
        WristBody = []
        for i in range(0, 4):   
            b = [[1,1],[1,-1],[-1,1],[-1,-1]]
            if b[i][0]==1:
                gripper_width = gripper_width_top
            else:
                gripper_width = gripper_width_bot
            #~Finger Body (changes with opening)
            WristBody.append(tcp_pos+0*hand_Z +b[i][0]*(gripper_width)*hand_X+(b[i][1]*(gripper_length/2.0)+fing_offset)*hand_Y)
            WristBody.append(tcp_pos+dist_tcp_to_intersection*hand_Z+b[i][0]*(gripper_width)*hand_X+(b[i][1]*(gripper_length/2.0)+fing_offset)*hand_Y)
        return np.asarray(WristBody)
        
def plotBoundBoxes(fingerPoints, wristPoints, tubePoints, listener, br):
        Tote = getToteSections(listener=listener, br=br);
#        storage_points = getStoragePoints()

        ax = buildPlt()
        updatePlt(fingerPoints, ax, edgeCol = 'b', lineColor = 'b')
        updatePlt(wristPoints, ax, edgeCol = 'b', lineColor = 'b')
#        updatePlt(tubePoints, ax, edgeCol = 'b', lineColor = 'b')
#        updatePlt(storage_points, ax, edgeCol = 'r', lineColor = 'r')

        #~ updatePlt(toteBody1, ax, edgeCol = 'r', lineColor = 'b')
        #~ updatePlt(toteBody2, ax, edgeCol = 'r', lineColor = 'b')
            #~ ax = buildPlt()
        for lv1 in range(0,7):
            updatePlt(Tote[lv1], ax, edgeCol = 'r', lineColor = 'b')
    #~ destroyPlt()
    
        destroyPlt()
        
def axisEqual3D(ax):
    extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
    sz = extents[:,1] - extents[:,0]
    centers = np.mean(extents, axis=1)
    maxsize = max(abs(sz))
    r = maxsize/2
    for ctr, dim in zip(centers, 'xyz'):
        getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)
        
def buildPlt():
    fig = plt.figure()
    ax = fig.gca(projection = '3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    return ax
        
def destroyPlt():
    plt.show()
    plt.close()

def updatePlt(pts, ax, edgeCol = 'r', lineColor = 'b'):
    plt.axis('equal')
    ax.set_aspect('equal')
    hull=ConvexHull(pts)
    # 12 = 2 * 6 faces are the simplices (2 simplices per square face)
    line_color = lineColor + 'o'
    edge_col = edgeCol + '-'
    for s in hull.simplices:
            s = np.append(s, s[0])  # Here we cycle back to the first coordinate
            ax.plot(pts[s, 0], pts[s, 1], pts[s, 2], edge_col)
    ax.plot(pts.T[0], pts.T[1], pts.T[2], line_color)
    axisEqual3D(ax)

def collisionFree(objInput, binId, listener, br, finger_opening=0, safety_margin= 0., theta = 0.):
    #~ print 'objInput', objInput
    #~get finger points
    T = ik.helper.matrix_from_xyzquat(objInput)
    hand_orient_base = np.array(T[0:3][0:3])
    finger_pts_3d = getFingerPoints(finger_opening, [0,0,0], hand_orient_base, False)
    #~ get bin points
    bin_pts_3d = getBinPoints(binId=binId, listener=listener, br=br)
    #~convert to 2d
    bin_pts = bin_pts_3d[:,0:2]
    finger_pts = finger_pts_3d[:,0:2]
    #~desired target position
    #~ target_wf=np.array(objInput[0:2])
    target_wf = np.array(objInput[0:2],ndmin=2)
    target_hf=np.array([0,0],ndmin=2)
    #~function properties
    show_plot = False
    margin=0
    (shape_translation,dist_val_min,feasible_solution,nearest_point) = suction_projection_func(bin_pts,finger_pts,target_wf,target_hf,theta,show_plot,margin)
    return shape_translation
        
if __name__=='__main__':

    rospy.init_node('collision_helper', anonymous=True) 
    goToHome.prepGripperPicking()
#    unit_test(finger_opening = 0.0)
    print '[Collision detection] End of program'
    

