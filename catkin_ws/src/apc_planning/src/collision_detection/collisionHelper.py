#~ from ik.ik import *
import sys
import os
rgrasp_path = os.environ['RGRASP_BASE'] + '/catkin_ws/src/apc_planning/src'
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
from manual_fit.srv import *
import tf
#import collision_free_placing 
import gripper

def getToteSections(listener, br, flag = True):
        #~build outside sections and floor
        world_X, world_Y, world_Z, tote_X,tote_Y,tote_Z, tote_pose_pos = reference_frames(listener = listener, br=br)
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
    bin_center_pose = get_params_yaml('bin'+str(binId)+'_pose')
    bodies.append(buildCenteredBox(length=xlength-tol*2, width=ywidth-tol*2, heigth=zheight, midPos=bin_center_pose[0:3], listener=listener, br=br))

    return np.asarray(bodies[0])
    

def buildCenteredBox(length, width, heigth, midPos, listener, br):
    world_X, world_Y, world_Z, tote_X,tote_Y,tote_Z, tote_pose_pos = ik.helper.reference_frames(listener=listener, br=br)
    #Build center points
    xVec = np.array([])
    #
    TotePoints = []
    TotePointsExtended = []
    ToteBody = []
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

def buildBoxes(V1, V2):
        Box1 = []
        Box2 = []
        Box1.append(V1[:,0])
        Box1.append(V1[:,1])
        Box1.append(V1[:,2])
        Box2.append(V2[:,0])
        Box2.append(V2[:,1])
        Box2.append(V2[:,2])
        return Box1, Box2
        
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

def checkPointCollision(Body1, Body2):
    #~ print ToteBody 
    collision=False
    for i in range(0, len(Body1)):  
        if in_hull(Body1[i], Body2):
            collision = True
    return (collision)
    
def in_hull(p, hull):
    from scipy.spatial import Delaunay
    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)
    return hull.find_simplex(p)>=0
    
    #return (collision, tcp_pos[0:2], hand_orient_norm_collision_free)

def checkCollisionGJK(points1, points2, numIt = 6, flag=0):
        #~ Build boxes from convex cloud of points
        Box1, Box2 = buildBoxes(points1, points2)
        collisionFlag = GJK(Box1, Box2, 6)
        if flag == 1:
                plotBoxes(points1, points2)
        if collisionFlag == 1:
            collisionFlagBool = True
        else:
            collisionFlagBool = False
        return collisionFlagBool
        
def GJK(shape1, shape2, iterations):

    #%Point 1 and 2 selection (line segment)
    v = np.array(np.hstack((0.8, 0.5, 1.)))
    [a, b] = pickLine(v, shape2, shape1)

    #%Point 3 selection (triangle)
    [a, b, c, flag] = pickTriangle(a, b, shape2, shape1, iterations)
    
    #%Point 4 selection (tetrahedron)
    if flag == 1:
        #%Only bother if we could find a viable triangle.
        [a, b, c, d, flag] = pickTetrahedron(a, b, c, shape2, shape1, iterations)

        #~ print 'd', d
        #~ print 'flag', flag
    return flag
    
def pickLine(v, shape1, shape2):

    # Local Variables: a, shape2, shape1, b, v
    # Function calls: pickLine, support
    #%Construct the first line of the simplex
    b = support(shape2, shape1, v)
    a = support(shape2, shape1, (-v))
    return a, b
    
def pickTriangle(a, b, shape1, shape2, IterationAllowed):

    # Local Variables: a, c, b, abc, i, flag, ao, abp, shape2, shape1, IterationAllowed, v, ab, acp, ac
    # Function calls: support, cross, dot, pickTriangle
    flag = 0.
    #%So far, we don't have a successful triangle.
    #%First try:
    ab = b-a
    ao = -a
    v = np.cross(np.cross(ab, ao), ab)
    #% v is perpendicular to ab pointing in the general direction of the origin.
    c = b
    b = a
    a = support(shape2, shape1, v)
    for i in range(0, IterationAllowed):
        #%iterations to see if we can draw a good triangle
        #Time to check if we got it:
                ab = b-a
                ao = -a
                ac = c-a
                #Normal to face of triangle
                abc = np.cross(ab,ac)
                #Perpendicular to AB going away from triangle
                abp = np.cross(ab,abc)
                #Perpendicular to AC going away from triangle
                acp = np.cross(abc,ac)
        #%Allowing 10 tries to make a good tetrahedron.
                #Is origin above (outside) AB?   
                if np.dot(abp,ao) > 0:
                        c = b #Throw away the furthest point and grab a new one in the right direction
                        b = a
                        v = abp #cross(cross(ab,ao),ab);
                        #Is origin above (outside) AC?
                elif np.dot(acp, ao) > 0:
                        b = a
                        v = acp #cross(cross(ac,ao),ac);
                else:
                        flag = 1
                        break #We got a good one.
                a = support(shape2,shape1,v)
    return a, b, c, flag
    
def pickTetrahedron(a, b, c, shape1, shape2, IterationAllowed):

        # Local Variables: a, c, b, d, acd, i, flag, abc, shape2, shape1, IterationAllowed, adb, ao, v, ab, ac, ad
        # Function calls: dot, support, cross, pickTetrahedron
        #%Now, if we're here, we have a successful 2D simplex, and we need to check
        #%if the origin is inside a successful 3D simplex.
        #%So, is the origin above or below the triangle?
        flag = 0
        ab = b-a
        ac = c-a

        #%Normal to face of triangle
        abc = np.cross(ab, ac)
        ao = -a
        
        if np.dot(abc, ao) > 0.:
                #%Above
                d = c
                c = b
                b = a
                v = abc
                a = support(shape2, shape1, v)
    #%Tetrahedron new point
        else:
                #%below
                d = b
                b = a
                v = -abc
                a = support(shape2, shape1, v)
                #%Tetrahedron new point
        
        for i in range(0, IterationAllowed):
                #Check the tetrahedron:
                ab = b-a
                ao = -a
                ac = c-a
                ad = d-a
                ad = d-a

                #We KNOW that the origin is not under the base of the tetrahedron based on
                #the way we picked a. So we need to check faces ABC, ABD, and ACD.
                #Normal to face of triangle
                abc = np.cross(ab,ac)
                if np.dot(abc, ao) > 0: #Above triangle ABC
                        #No need to change anything, we'll just iterate again with this face as
                        #default.
                        pass #do nothing
                else:
                        acd = np.cross(ac,ad)#Normal to face of triangle

                        if np.dot(acd, ao) > 0:#Above triangle ACD
                                #Make this the new base triangle.
                                b = c
                                c = d
                                ab = ac
                                ac = ad         
                                abc = acd  
                        else:
                                adb = np.cross(ad,ab)#Normal to face of triangle
                                
                                if np.dot(adb, ao) > 0: #Above triangle ADB
                                        #Make this the new base triangle.
                                        c = b
                                        b = d              
                                        ac = ab
                                        ab = ad
                                        abc = adb     
                                else:
                                        flag = 1
                                        break#It's inside the tetrahedron.
                
                # try again:
                if np.dot(abc, ao) > 0: # Above
                        d = c
                        c = b
                        b = a    
                        v = abc
                        a = support(shape2,shape1,v) #Tetrahedron new point
                else: #below
                        d = b
                        b = a 
                        v = -abc
                        a = support(shape2,shape1,v) #Tetrahedron new point
                        
        return [a, b, c, d, flag]
        
def getFarthestInDir(shape, v):
    # Local Variables: rowIdxSet, dotted, point, colIdx, rowIdx, shape, YData, XData, v, maxInCol, maxInRow, ZData
    # Function calls: max, getFarthestInDir
    #%Find the furthest point in a given direction for a shape
    XData = shape[0]
    #%get(shape,'XData'); % Making it more compatible with previous MATLAB releases.
    YData = shape[1]
    #%get(shape,'YData');
    ZData = shape[2]
    #%get(shape,'ZData');
    dotted = np.dot(XData, v[0])+np.dot(YData, v[1])+np.dot(ZData, v[2])
    [maxInd, Ind] = dotted.max(0),dotted.argmax(0) #matcompat.max(dotted)
    #~ [maxInRow, colIdx] = maxInCol.max(0),maxInCol.argmax(0)
    #~ rowIdx = rowIdxSet[int(colIdx)]
    point = np.array(np.hstack((XData[int(Ind)], YData[int(Ind)], ZData[int(Ind)])))
    return point

def support(shape1,shape2,v):
        #Support function to get the Minkowski difference.
        point1 = getFarthestInDir(shape1, v)
        point2 = getFarthestInDir(shape2, -v)
        point = point1 - point2
        return point
        
def collisionFree(objInput, binId, listener, br, finger_opening=0, safety_margin= 0., theta = 0.):
    #~ print 'objInput', objInput
    #~get finger points
    tcp_pos = objInput[0:3]
    hand_orient_quat = objInput[3:7]
    base_pose = [0,0,0, 0, 1, 0, 0]
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
    #~desired orientation
    #~ theta=0
    #~function properties
    show_plot = False
    margin=0
    (shape_translation,dist_val_min,feasible_solution,nearest_point) = suction_projection_func(bin_pts,finger_pts,target_wf,target_hf,theta,show_plot,margin)
    return shape_translation
        
def unit_test(finger_opening):
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    #~initi
    objInput = []
    plans = []
    #~bin 0
    objInput.append([0.96875679493, -0.3314011693, 0.320692062378, -0.70710678118654757, -0.70710678118654757, -0.0, -0.0])
    objInput.append([0.629658937454, -0.606883049011, 0.320692062378, -0.0, 1.0, 0.0, 0.0])
    
    objInput.append([1.01049637794, -0.837368607521, 0.320692062378, 0.70710678118654757, -0.70710678118654757, -0.0, 0.0])
    objInput.append([ 1.30534303188, -0.569402635098, 0.320692062378, 1.0, -0.0, 0., -0.0])
    
    #~bin 1
    objInput.append([0.980982005596, 0.0359762609005, 0.320692062378, -0.70710678118654757, -0.70710678118654757, -0.0, -0.0])
    objInput.append([0.609354317188, -0.198960453272, 0.320692062378, -0.0, 1.0, 0.0, 0.0])

    objInput.append([1.01049637794, -0.837368607521, 0.320692062378, 0.70710678118654757, -0.70710678118654757, -0.0, 0.0])
    objInput.append([1.33754169941, -0.234649211168, 0.320692062378, 1.0, -0.0, 0.0, -0.0])
    
    #~bin 2
    objInput.append([0.992403447628, 0.379200667143, 0.283088296652, -0.70710678118654757, -0.70710678118654757, -0.0, -0.0])
    objInput.append([0.663011491299, 0.186915069818, 0.283088296652, 0.0, 1.0, 0.0, 0.0])

    objInput.append([1.01049637794, -0.837368607521, 0.283088296652, 0.699359357357, -0.714531302452, -0.0172835830599, 0.00658781733364])
    objInput.append([1.37730908394, 0.178792119026, 0.283088296652, 1.0, -0.0, 0.0, -0.0])
    
    #~bin 3
    objInput.append([1.00729203224, 0.826616168022, 0.4350451231, -0.70710678118654757, -0.70710678118654757, -0.0, -0.0])
    objInput.append([0.719842135906, 0.597367167473, 0.4350451231, 0.0, 1.0, 0.0, 0.0])

    objInput.append([1.01049637794, -0.837368607521, 0.4350451231, 0.699359357357, -0.714531302452, -0.0172835830599, 0.00658781733364])
    objInput.append([1.26739788055, 0.614399254322, 0.4350451231, 1.0, -0.0, 0.0, -0.0])
    
    #~bin 4
    objInput.append([1.11292684078, -0.380549579859, 0.4350451231, -0.70710678118654757, -0.70710678118654757, -0.0, -0.0])
    objInput.append([0.555991172791, -0.587136209011, 0.4350451231, 0.0, 1.0, 0.0, 0.0])

    objInput.append([1.1149418354, -0.839584231377, 0.4350451231, 0.70710678118654757, -0.70710678118654757, -0.0, 0.0])
    objInput.append([1.429666996, -0.618811905384, 0.4350451231, 1.0, -0.0, 0., -0.0])
    
    #~bin 5
    objInput.append([0.854664802551, -0.371939957142, 0.4350451231, -0.70710678118654757, -0.70710678118654757, -0.0, -0.0])
    objInput.append([0.555991172791, -0.587136209011, 0.4350451231, 0.0, 1.0, 0.0, 0.0])

    objInput.append([0.880754649639, -0.834616422653, 0.4350451231, 0.699359357357, -0.714531302452, -0.0172835830599, 0.00658781733364])
    objInput.append([-0.565601050854, -0.572955012321, 0.4350451231, 1.0, 0.0, 0.0, 0.0])
    
    #~bin 6
    objInput.append([0.111314751208, -0.480122387409, 0.624268651009, -0.70710678118654757, -0.70710678118654757, -0.0, -0.0])
    objInput.append([-0.565601050854, -0.572955012321, 0.624268651009, 0.0, 1.0, 0.0, 0.0])

    objInput.append([0.117065243423, -0.822051107883, 0.624268651009, 0.70710678118654757, -0.70710678118654757, -0.0, -0.0])
    objInput.append([0.244894966483, -0.625436663628, 0.624268651009, 1.0, -0.0, 0., -0.0])
    
    #~bin 7
    objInput.append([-0.20037125051, -0.403191208839, 0.624268651009, -0.70710678118654757, -0.70710678118654757, -0., -0.])
    objInput.append([-0.565601050854, -0.572955012321, 0.624268651009, 0.0, 1.0, 0.0, 0.0])

    objInput.append([-0.171951562166, -0.815774440765, 0.624268651009, 0.70710678118654757, -0.70710678118654757, -0.0, -0.0])
    objInput.append([0.244894966483, -0.625436663628, 0.624268651009, 1.0, -0.0,0.0, -0.0])
    
    #~bin 8
    objInput.append([-0.0678554922342, 0.848172545433, 0.718854248524, -0.70710678118654757, -0.70710678118654757, -0.0, -0.0])
    objInput.append([-0.406745672226, 0.651868224144, 0.718854248524, 0.0, 1.0, 0.0, 0.0])

    objInput.append([-0.0654902458191, 0.323144733906, 0.718854248524, 0.70710678118654757, -0.70710678118654757, -0.0, -0.0])
    objInput.append([0.367074251175, 0.644799649715, 0.718854248524, 1.0, -0.0, 0.0, -0.0])

    #~define gripper transform
    q_initial = [-0.0014,    0.2129,    0.3204,    0,    1.0374,   -0.0014] # same q as goToARC
    spatula_tip_to_tcp_dist=rospy.get_param("/gripper/spatula_tip_to_tcp_dist")
    l1 = 0.0
    l2 = 0.0
    l3 = 0.0
    tip_hand_transform = [l1, l2, l3, 0,0,0]
    #~############################################################################
    bin_list = [0,1,2,3]#,4,5,6,7,8]
    wall_list = [0,1,2,3]
    counter = 0
        
    #~Execute plans
    goToHome.goToARC()
    
    for binId in bin_list:
        print 'binId', binId
        for wall in wall_list:
            print 'wall', wall
            #~generate plans
            colFreePose = collisionFree(objInput=objInput[counter], binId=binId, listener=listener, br=br, finger_opening=finger_opening)
            target = np.zeros((3))
            target[0:2] = colFreePose[0:2]
            target[2] = objInput[counter][2]
            gripperQuat = objInput[counter][3:7]
            #~Go above bin
            plan, qf, plan_possible = ik.ik.generatePlan(q_initial, target + np.array([0,0,0.3]) , gripperQuat, tip_hand_transform, 'fastest')
            if plan_possible:
                plans.append(plan) 
                q_initial = qf
            else:
                return '[IK] Error'  
            #~Go above bin
            plan, qf, plan_possible = ik.ik.generatePlan(q_initial, target + np.array([0,0,0.1]) , gripperQuat, tip_hand_transform, 'fastest')
            if plan_possible:
                plans.append(plan) 
                q_initial = qf
            else:
                return '[IK] Error'  
            #~Go to point of interest
            plan, qf, plan_possible = ik.ik.generatePlan(q_initial, target , gripperQuat, tip_hand_transform, 'fastest', guard_on=ik.ik.WeightGuard(binId))
            if plan_possible:
                plans.append(plan) 
                q_initial = qf
            else:
                return '[IK] Error'
            counter += 1

            #~ gripper.close()
            
            ik.ik.executePlanForward(plans, False)
        
            plans = []
#        q_initial = collision_free_placing(binId, listener, isSuction = False, isReturn=True)
#        if binId==8:
#            collision_free_placing.go_arc_safe()
#        else:
#            q_initial = collision_free_placing.collision_free_placing(binId+1, listener, isSuction = False, isReturn=True)

if __name__=='__main__':

    rospy.init_node('collision_helper', anonymous=True) 
    #~ unit_test()
    goToHome.prepGripperPicking()
    #~ gripper.open()
    unit_test(finger_opening = 0.0)
    print '[Collision detection] End of program'
    
    #~ finger_opening=0.11
    #~ tcp_pos = [1,0,0]
    #~ hand_orient_norm = np.identity(3)
    #~ isSuction = True
    #~ fingerPoints= getFingerPoints(finger_opening, tcp_pos, hand_orient_norm, isSuction)
    #~ wristPoints = getWristPoints(tcp_pos, hand_orient_norm, isSuction)
    
    #~ finger_opening=0.11
    #~ tcp_pos = [1,0,0]
    #~ hand_orient_norm = np.identity(3)
    #~ isSuction = True
    #~ fingerPoints = getFingerPoints(finger_opening, tcp_pos, hand_orient_norm, isSuction)
    

    #~ fingerPoints= getFingerPoints(finger_opening, tcp_pos, hand_orient_norm, isSuction)
    #~ wristPoints = getWristPoints(tcp_pos, hand_orient_norm, isSuction)
    #~Plot collision boxes (Optional)
    #~ listener = tf.TransformListener()
    #~ rospy.sleep(0.5)
    #~ b=[]
    #~ ax = buildPlt()
    #~ for i in range(0,9):
        #~ b.append(getBinPoints(i, listener = listener))
        #~ if i<4:
            #~ updatePlt(b[i], ax, edgeCol = 'b', lineColor = 'b')
        #~ else:
            #~ updatePlt(b[i], ax, edgeCol = 'r', lineColor = 'r')
    #~ destroyPlt()
    #~ storage_points = getStoragePoints()
    #~ print 'b3', b3
    #~ print 'storage_points', storage_points
    
    #~ ax = buildPlt()
    #~ updatePlt(fingerPoints, ax, edgeCol = 'r', lineColor = 'b')
    #~ updatePlt(wristPoints, ax, edgeCol = 'r', lineColor = 'b')
    
    #~ updatePlt(totePointsList[0], ax, edgeCol = 'b', lineColor = 'b')
    #~ updatePlt(totePointsList[1], ax, edgeCol = 'b', lineColor = 'b')
    #~ updatePlt(totePointsList[2], ax, edgeCol = 'b', lineColor = 'b')
    #~ updatePlt(totePointsList[3], ax, edgeCol = 'b', lineColor = 'b')
    #~ updatePlt(totePointsList[4], ax, edgeCol = 'b', lineColor = 'b')
    #~ updatePlt(totePointsList[5], ax, edgeCol = 'b', lineColor = 'b')
    #~ updatePlt(totePointsList[6], ax, edgeCol = 'b', lineColor = 'b')
    #~ updatePlt(b0, ax, edgeCol = 'b', lineColor = 'b')
    #~ updatePlt(b1, ax, edgeCol = 'b', lineColor = 'b')
    #~ updatePlt(b2, ax, edgeCol = 'b', lineColor = 'b')
    #~ updatePlt(b3, ax, edgeCol = 'b', lineColor = 'b')
    #~ updatePlt(storage_points, ax, edgeCol = 'r', lineColor = 'r')
    #~ destroyPlt()
    #~ fingerPoints = getFingerPoints(finger_opening, tcp_pos, hand_orient_norm, isSuction)
    #~ wristPoints = getWristPoints(tcp_pos, hand_orient_norm, isSuction)
    #~ binPoints = getBinPoints(0)
    
    #~ ax = buildPlt()
    #~ updatePlt(fingerPoints, ax, edgeCol = 'r', lineColor = 'b')
    #~ updatePlt(wristPoints, ax, edgeCol = 'r', lineColor = 'b')
    #~ updatePlt(wristPoints, ax, edgeCol = 'r', lineColor = 'b')
    #~ destroyPlt()
        
    #~ isExecute = True
    #~ posesrv = rospy.ServiceProxy('/manualfit_object_pose', GetPose)   # should move service name outsoft_white_lightbulb
    #~ rospy.sleep(0.5)
    #~ data = posesrv('','')
    #~ objInput = pose2list(data.pa.object_list[0].pose)
    
    #~ objInput = [1.01179456711, -0.699, -0.0429569929838, 0.0, -0.707106769085, 0.0, 0.707106769085]
    #~ objInput = [ 1.01179456711, -0.441, -0.0429569780827, 0.0, -0.707106769085, 0.0, 0.707106769085]
    #~ objInput = [1.196, -0.594460308552, -0.0429569743574, 0.506816148758, -0.493089616299, 0.506816267967, 0.493089616299]
    #~ objInput = [0.748, -0.606691658497, -0.0429570153356, 0.506816148758, -0.493089616299, 0.506816267967, 0.493089616299]
    
    #~ objInput = [1.01179456711, 0.699+0.006, -0.0429569929838, 0.0, -0.707106769085, 0.0, 0.707106769085]
    #~ objInput = [ 1.01179456711, -0.441, -0.0429569780827, 0.0, -0.707106769085, 0.0, 0.707106769085]
    #~ objInput = [1.196, -0.594460308552, -0.0429569743574, 0.506816148758, -0.493089616299, 0.506816267967, 0.493089616299]
    #~ objInput = [0.748, -0.606691658497, -0.0429570153356, 0.506816148758, -0.493089616299, 0.506816267967, 0.493089616299]
    #~ objInput_warped = pose_warp(objInput, 0)
    #~ print 'objInput', objInput
    #~ print 'objInput_warped', objInput_warped
    #~ TARGET = 'expo_dry_erase_board_eraser'

    #~ for x in range(0,1):
        #~ (output_dict)=pick(objInput_warped,
                #~ isExecute,
                #~ objId = TARGET,
                #~ binId = 0,
                #~ flag = 0,
                #~ withPause = True)

