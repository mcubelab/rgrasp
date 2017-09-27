#!/usr/bin/env python
import numpy as np
from scipy.optimize import least_squares
import tf.transformations as tfm
import cv2
from numpy import array as npa
import numpy as np
import os, json
import ik.helper
import sys


def quat_to_rod(q):
    # q = [qx, qy, qz, qw]
    rot = tfm.quaternion_matrix(q)[0:3][:, 0:3]
    dst, jacobian = cv2.Rodrigues(rot)
    return dst.T.tolist()[0]

def mat_to_rod(rotmat):
    dst, jacobian = cv2.Rodrigues(rotmat[0:3][:, 0:3])
    return dst.T.tolist()[0]


def rod_to_quad(r):
    # q = [qx, qy, qz, qw]
    rotmat , jacobian = cv2.Rodrigues(npa(r))
    rotmat = np.append(rotmat, [[0,0,0]], 0)
    rotmat = np.append(rotmat, [[0],[0],[0],[1]], 1)
    q = tfm.quaternion_from_matrix(rotmat)
    return q.tolist()

def rod_to_mat(r):
    # q = [qx, qy, qz, qw]
    rotmat , jacobian = cv2.Rodrigues(npa(r))
    rotmat = np.append(rotmat, [[0,0,0]], 0)
    rotmat = np.append(rotmat, [[0],[0],[0],[1]], 1)
    return rotmat

class Program:
    def __init__(self, point3Ds, point2Ds, x0, x0_int):
        self.point3Ds = point3Ds
        self.point2Ds = point2Ds
        self.x0 = x0
        self.x0_int = x0_int

    def obj_func(self, cam):

        fx, fy = x0_int[0], x0_int[1]
        cx, cy = x0_int[2], x0_int[3]
        distCoeffs = (0.0, 0.0, 0.0, 0.0, 0.0)   ## hack no distortion
        tvec = cam[0:3]  # x,y,z
        rvec = cam[3:6]  # rodrigues

        # project
        #point2Ds_p = cv.project(point3Ds, cam)
        cameraMatrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        point2Ds_p, jacobian = cv2.projectPoints(npa(self.point3Ds, dtype=np.float), rvec, tvec, cameraMatrix, distCoeffs)
        #print point2Ds_p
        point2Ds_pp = [list(p[0]) for p in point2Ds_p]
        diff = npa(point2Ds_pp, dtype=np.float) - npa(self.point2Ds, dtype=np.float)
        diff = diff.flatten(1)
        #import pdb;
        #pdb.set_trace()
        #print diff
        res = np.linalg.norm(diff)
        print res / len(self.point3Ds)
        return diff

    def run(self):

        res_1 = least_squares(self.obj_func, self.x0)

        print '--original--'
        trans =  self.x0[0:3]
        rod =  self.x0[3:6]
        q = rod_to_quad(rod)
        print 'pose', list(trans) + list(q)

        print '\n--optimized--'
        trans = res_1.x[0:3]
        rod = res_1.x[3:6]
        q = rod_to_quad(rod)
        print 'pose', list(trans) + list(q)

        transform = tfm.concatenate_matrices(tfm.translation_matrix(trans), tfm.quaternion_matrix(q))
        inversed_transform = tfm.inverse_matrix(transform)
        translation = tfm.translation_from_matrix(inversed_transform)
        quaternion = tfm.quaternion_from_matrix(inversed_transform)
        pose =  translation.tolist() + quaternion.tolist()
        #print 'webcam_T_robot:', " ".join('%.8e' % x for x in pose), '\n'
        print 'matrix:\n', np.linalg.inv(ik.helper.matrix_from_xyzquat(pose))

        #print 'K: ', [res_1.x[0], 0.0, res_1.x[2], 0.0, res_1.x[1], res_1.x[3], 0.0, 0.0, 1.0]
        #print 'P: ', [res_1.x[0], 0.0, res_1.x[2], 0.0, 0.0, res_1.x[1], res_1.x[3], 0.0, 0.0, 0.0, 1.0, 0.0]

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
    invmat = np.linalg.inv(np.array(ret).reshape((4,4)))
    rod = mat_to_rod(invmat)
    trans = [invmat[0,3], invmat[1,3], invmat[2,3]]
    return trans + rod

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


if __name__ == '__main__':
    color1 = (0,255,255)
    color2 = (0,0,255)
    color3 = (255,0,255)


    cam_configs = {'612203002922': {'bin_num': 'bin0', 'place': 'passive_near', 'ns': 'arc_1'},
              '614203000465': {'bin_num': 'bin0', 'place': 'passive_far', 'ns': 'arc_1'},
              #'612203004574': {'bin_num': 'bin0', 'place': 'active_near', 'ns': 'arc_1'},
              #'616205005772': {'bin_num': 'bin0', 'place': 'active_far', 'ns': 'arc_1'},

              '616205001219': {'bin_num': 'bin1', 'place': 'passive_near', 'ns': 'arc_1'},
              '61420501085': {'bin_num': 'bin1', 'place': 'passive_far', 'ns': 'arc_1'},
              '616205004776': {'bin_num': 'bin1', 'place': 'active_near', 'ns': 'arc_1'},
              '614203003651': {'bin_num': 'bin1', 'place': 'active_far', 'ns': 'arc_1'},

              '612205002211': {'bin_num': 'bin2', 'place': 'passive_near', 'ns': 'arc_2'}, 
              '612203004574': {'bin_num': 'bin2', 'place': 'passive_far', 'ns': 'arc_2'}, 
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

    argv = sys.argv
    if argv[1] in ['bin0', 'bin1', 'bin2', 'bin3']:
        cameraids = [camid for camid, cam_config in cam_configs.iteritems() if cam_config['bin_num'] == argv[1]]
    else:
        cameraids = [argv[1]] #'612203002922'
        
    for cameraid in cameraids:

        print 'calibrating', cameraid
        save_dir = os.environ["ARCDATA_BASE"] + "/camera_calib/" + cameraid + '/'
        x0_int = get_int(cameraid)


        x0_ext = get_ext(cameraid)
        print x0_ext

        #x0_int = [6.16917053e+02, 6.16917175e+02, 3.16593872e+02, 2.36076706e+02, 2.671554e-02, 6.672619e-01, -6.263159e-03, 6.014189e-04, -2.923799e+00]

        x0 = x0_ext
        with open(save_dir + 'data.extracted2d.json') as data_file:
            data = json.load(data_file)

        point3d = [d["cross3d"][0:3] for d in data]
        point2d = [d["cross2d"] for d in data]
        #print point3d
        p = Program(point3d, point2d, x0, x0_int)
        cam = p.run()

        # show reprojection
        fx, fy = x0_int[0], x0_int[1]
        cx, cy = x0_int[2], x0_int[3]
        distCoeffs = (0.,0.,0.,0.,0.)
        tvec = cam[0:3]  # x,y,z
        rvec = cam[3:6]  # rodrigues
        #tvec = x0[0:3]
        #rvec = x0[3:6]

        # project
        cameraMatrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

        point2Ds_p_nd, jacobian = cv2.projectPoints(npa(point3d, dtype=np.float), rvec, tvec, cameraMatrix, (0.,0.,0.,0.,0.))

        for i, d in enumerate(data):
            image_viz = cv2.imread(save_dir + d['pic_path'])


            pt_int = tuple([int(round(p)) for p in d['cross2d']])
            cv2.line(image_viz, (pt_int[0]-2, pt_int[1]), (pt_int[0]+2, pt_int[1]), color1)
            cv2.line(image_viz, (pt_int[0], pt_int[1]-2), (pt_int[0], pt_int[1]+2), color1)

            pt_int = tuple([int(round(p)) for p in point2Ds_p_nd[i][0]])
            cv2.line(image_viz, (pt_int[0]-2, pt_int[1]), (pt_int[0]+2, pt_int[1]), color3)
            cv2.line(image_viz, (pt_int[0], pt_int[1]-2), (pt_int[0], pt_int[1]+2), color3)

            cv2.imshow("image", image_viz)

            while True:
                # display the image and wait for a keypress
                key = cv2.waitKey(3) & 0xFF
                if key == ord("n"):
                    break

        save_ext(cameraid, cam)
