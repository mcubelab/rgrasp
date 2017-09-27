#!/usr/bin/env python
import numpy as np
from scipy.optimize import least_squares
import tf.transformations as tfm
import cv2
from numpy import array as npa
import numpy as np
import os, json
import sys


def matrix_from_xyzrpy(arg1, arg2=None):
    if arg2 is not None:
        translate = arg1
        rpy = arg2
    else:
        translate = arg1[0:3]
        rpy = arg1[3:6]

    return np.dot(tfm.compose_matrix(translate=translate) ,
                   tfm.euler_matrix(rpy[0],rpy[1],rpy[2])).tolist()

class Program:
    def __init__(self, point3Ds, point3Ds_pc, x0, x0_ext, x0_int):
        self.point3Ds = point3Ds
        self.point3Ds_pc = point3Ds_pc
        self.x0 = x0
        self.x0_ext = x0_ext
        self.x0_int = x0_int

    def obj_func(self, cam):
        c0 = cam[0]
        c1 = cam[1]
        c2 = cam[2]
        diff = []
        
        #print matrix_from_xyzrpy([0,0,0, c,0,0])
        for i in xrange(len(self.point3Ds)):
            point3D = self.point3Ds[i]
            point3D_pc = self.point3Ds_pc[i]
            #point3D_pc_trans = np.array(point3D_pc + [1.0]) * c
            #import ipdb; ipdb.set_trace()
            point3D_pc_trans = np.dot(matrix_from_xyzrpy([c2 * point3D_pc[0] / point3D_pc[2],c2 * point3D_pc[1] / point3D_pc[2],c2, 0,0,0]), np.array([point3D_pc[0], point3D_pc[1], point3D_pc[2], 1.0]) )
            
            #point3D_pc_trans[0] = point3D_pc[0] + c * point3D_pc[0] / point3D_pc[2]
            #point3D_pc_trans[1] = point3D_pc[1] + c * point3D_pc[1] / point3D_pc[2]
            #point3D_pc_trans[2] = point3D_pc[2] + c 
            
            point3D_pc_world = np.dot(self.x0_ext, point3D_pc_trans)
            #print point3D, point3D_pc_world[0:3].tolist()
            diff_ = npa(point3D, dtype=np.float) - point3D_pc_world[0:3].tolist()
            diff.extend(diff_.tolist())
        #import ipdb; ipdb.set_trace()
        res = np.linalg.norm(diff)
        print res / len(self.point3Ds)
        err = res / len(self.point3Ds)
        return err
        

    def run(self):

        res_1 = least_squares(self.obj_func, self.x0)

        print '--original--'
        print self.x0

        print '\n--optimized--'
        print res_1.x

        return res_1.x


def get_numbers_from_file(filename):
    ret = []
    with open(filename) as input_file:
        for line in input_file:
            line = line.strip()
            for number in line.split():
                ret.append( float(number) )
    return ret


def get_int(cameraid):
    ret = get_numbers_from_file(os.environ['ARC_BASE'] + '/catkin_ws/src/passive_vision/camerainfo/' + cameraid + '.intrinsics.txt')
    print ret
    return [ret[0], ret[4], ret[2], ret[5], 0,0,0,0,0]
    
def get_ext(cameraid):
    ret = get_numbers_from_file(os.environ['ARC_BASE'] + '/catkin_ws/src/passive_vision/camerainfo/' + cameraid + '.pose.txt')
    
    print 'read mat: ', np.array(ret).reshape((4,4))
    return np.array(ret).reshape((4,4))

def save_ext(cameraid, cam):
    mat = rod_to_mat(cam[3:6])
    mat[0][3] = cam[0]
    mat[1][3] = cam[1]
    mat[2][3] = cam[2]
    filename = os.environ['ARC_BASE'] + '/catkin_ws/src/passive_vision/camerainfo/' + cameraid + '.pose.txt'
    mat = np.linalg.inv(mat)
    
    print 'saved mat: ', mat
    with open(filename, 'w') as output_file:
        output_file.writelines(' '.join(str(j) for j in i) + '\n' for i in mat)


def save_depth_param(cameraid, cam):
    filename = os.environ['ARC_BASE'] + '/catkin_ws/src/passive_vision/camerainfo/' + cameraid + '.xyz.offset.txt'
    
    print 'saved param: ', cam
    with open(filename, 'w') as output_file:
        output_file.writelines(' '.join(str(j) for j in cam))

if __name__ == '__main__':
    color1 = (0,255,255)
    color2 = (0,0,255)
    color3 = (255,0,255)

    cam_configs = {'612203002922': {'bin_num': 'bin0', 'place': 'passive_near', 'ns': 'arc_1'},
              '614203000465': {'bin_num': 'bin0', 'place': 'passive_far', 'ns': 'arc_1'},
              '612203004574': {'bin_num': 'bin0', 'place': 'active_near', 'ns': 'arc_1'},
              '616205005772': {'bin_num': 'bin0', 'place': 'active_far', 'ns': 'arc_1'},

              '616205001219': {'bin_num': 'bin1', 'place': 'passive_near', 'ns': 'arc_1'},
              '61420501085': {'bin_num': 'bin1', 'place': 'passive_far', 'ns': 'arc_1'},
              '616205004776': {'bin_num': 'bin1', 'place': 'active_near', 'ns': 'arc_1'},
              '614203003651': {'bin_num': 'bin1', 'place': 'active_far', 'ns': 'arc_1'},

              '612205002211': {'bin_num': 'bin2', 'place': 'passive_near', 'ns': 'arc_2'}, 
              '612204001396': {'bin_num': 'bin2', 'place': 'passive_far', 'ns': 'arc_2'}, 
              '614205001856': {'bin_num': 'bin2', 'place': 'active_near', 'ns': 'arc_2'}, 
              '613201001839': {'bin_num': 'bin2', 'place': 'active_far', 'ns': 'arc_2'},

              '617205001931': {'bin_num': 'bin3', 'place': 'passive_near', 'ns': 'arc_2'},
              '612201002220': {'bin_num': 'bin3', 'place': 'passive_far', 'ns': 'arc_2'},
              '616203002024': {'bin_num': 'bin3', 'place': 'active_near', 'ns': 'arc_2'},
              '614204001012': {'bin_num': 'bin3', 'place': 'active_far', 'ns': 'arc_2'}}
    if len(sys.argv) < 2:
        print 'calib3d2d [camera serial number]\n'
        import pprint
        pp = pprint.PrettyPrinter(indent=4)
        pp.pprint(cam_configs)
        exit(0)

    cameraid = sys.argv[1] #'612203002922'

    save_dir = os.environ["ARCDATA_BASE"] + "/camera_calib/" + cameraid + '/'


    x0_ext = get_ext(cameraid)
    print x0_ext

    x0_int = get_int(cameraid)

    x0 = [0.0,0.0,0.0]
    with open(save_dir + 'data.extracted2d.json') as data_file:
        data = json.load(data_file)

    point3ds = []
    point2ds = []
    point3ds_pc = []
    depth_image = np.zeros((480,640,3), np.float32)
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg
    
    seqs = []
    for d in data:
        # read the file
        (x,y) = d["cross2d"]
        tmp = np.load(save_dir + d["pointcloud_path"])
        rgb_image = mpimg.imread(save_dir + d["pic_path"])
        point3d_ = tmp[int(y*3*640 + x*3) : int(y*3*640 + x*3 + 3)].tolist()
        
        #~ for i in xrange(480):
            #~ for j in xrange(640):
                #~ for k in [2]:
                    #~ depth_image[i][j][k] = tmp[int(i*3*640 + j*3 + 2)]
        
        #depth_image[int(y)][int(x)][0] = 5.0
        #blend_depth_image = rgb_image + depth_image
        #blend_depth_image[int(y)][int(x)][0] = 5.0
        #plt.imshow(blend_depth_image)
        #plt.show()
        #import ipdb; ipdb.set_trace()
        #raw_input('pause')
        
        if(np.linalg.norm(point3d_) < 1e-9):   # discard bad points
           continue
        point3ds_pc.append(point3d_)
        print point3d_
        
        point2ds.append(d["cross2d"])
        point3ds.append(d["cross3d"][0:3])
        seqs.append(d["pic_path"])
    p = Program(point3ds, point3ds_pc, x0, x0_ext, x0_int)
    cam = p.run()

    # show result
    
    c0 = cam[0]
    c1 = cam[1]
    c2 = cam[2]
    
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    point3Ds_pc_trans_world = []
    point3Ds_pc_world = []
    
    point3Ds_pc_cam = []
    point3Ds_cam = []
    point3D_pc_trans_cam = []
    for i in xrange(len(point3ds)):
        # read the file
        point3D_pc = point3ds_pc[i]
        point3D_pc_trans = np.dot(matrix_from_xyzrpy([c2 * point3D_pc[0] / point3D_pc[2],c2 * point3D_pc[1] / point3D_pc[2],c2, 0,0,0]), np.array([point3D_pc[0], point3D_pc[1], point3D_pc[2], 1.0]) )
            
        point3D_pc_trans_cam.append(point3D_pc_trans[0:3].tolist())
        point3D_pc_trans_world = np.dot(x0_ext, point3D_pc_trans)[0:3]
        point3Ds_pc_trans_world.append(point3D_pc_trans_world.tolist())
        point3D_pc_world = np.dot(x0_ext, np.array(point3D_pc +[1.0]))[0:3]
        point3Ds_pc_world.append(point3D_pc_world.tolist())
        
        point3D_cam = np.dot(np.linalg.inv(x0_ext), np.array(point3ds[i] +[1.0]))[0:3]
        point3Ds_cam.append(point3D_cam)
        
        print seqs[i], np.linalg.norm(np.array(point3D_cam)-np.array(point3D_pc))
            
    (xs, ys, zs) = zip(*point3ds)
    s1 = ax.scatter(xs, ys, zs, c='g')
    (xs, ys, zs) = zip(*point3Ds_pc_trans_world)
    s2 = ax.scatter(xs, ys, zs, c='r')
    (xs, ys, zs) = zip(*point3Ds_pc_world)
    s3 = ax.scatter(xs, ys, zs, c='b')
    
    ax.legend([s1, s2, s3], ['robot', 'point3Ds_pc_trans_world', 'point3Ds_pc_world'])
    
    #~ (xs, ys, zs) = zip(*point3Ds_cam)
    #~ s1 = ax.scatter(xs, ys, zs, c='g')
    #~ (xs, ys, zs) = zip(*point3ds_pc)
    #~ s2 = ax.scatter(xs, ys, zs, c='b')
    #~ (xs, ys, zs) = zip(*point3D_pc_trans_cam)
    #~ s3 = ax.scatter(xs, ys, zs, c='r')
    
    #~ ax.legend([s1, s2, s3], ['robot', 'point3Ds_pc_cam', 'point3D_pc_trans_cam'])
        
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()
    save_depth_param(cameraid, cam)
