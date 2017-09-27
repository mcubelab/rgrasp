from math import *
import copy
import random

from Mat.py import *

class RotMat(Mat):

    def __init__(self):
        Mat.__init__(0.0,3,3)
        for i in range(3):
            self.v[i][i] = 1.0

    def __init__(self, orig):
        Mat.__init__(orig)

    def __init__(self, vals):
        Mat.__init__(vals,3,3)

    def __init__(self, vals_str):
        Mat.__init__(vals_str, 3,3)

    def __init__(self, x, y, z):
        Mat.__init__(0.0,3,3)
        self.setRefFrame(x,y,z)

    def __init__(self, orig):
        Mat.__init__(0.0,3,3)
        if ((orignn==3) and (orig.mm==3)):
            for i in range(3):
                for j in range(3):
                    self.v[i][j] = orig[i][j]

# Eplicitly inherited for preserving the output class label
    def  __iadd__(self, orig):
        self = RotMat(self+=orig)
        return self

    def  __isub__(self, orig):
        self = RotMat(self-=orig)
        return self

    def  __imul__(self, const):
        self = RotMat(self*=const)
        return self
    
    def  __idiv__(self, const):
        self = RotMat(self/=const)
        return self

    def __add__(self, orig):
        return RotMat(self+orig)

    def __sub__(self, orig):
        return RotMat(self-orig)

    def __mul__(self, orig): # RotMat,Vec,double orig
        return RotMat(self*orig)

    def __div__(self, const):
        return RotMat(self/const)
    
    def setRefFrame(self, x, y, z):
        Mat.setCol(0, x)
        Mat.setCol(1, y)
        Mat.setCol(2, z)

    def rotX(alpha):
        self.v[0][0] = 1
        self.v[1][0] = 0
        self.v[2][0] = 0
        self.v[0][1] = 0
        self.v[1][1] = 0
        self.v[2][1] = cos(alpha)
        self.v[0][2] = 0
        self.v[1][2] = -sin(alpha)
        self.v[2][2] = cos(alpha)

    def rotY(self, alpha):
	self.v[0][0]=cos(alpha)
	self.v[1][0]=0
	self.v[2][0]=-sin(alpha)
	self.v[0][1]=0
	self.v[1][1]=1
	self.v[2][1]=0
	self.v[0][2]=sin(alpha)
	self.v[1][2]=0
	self.v[2][2]=cos(alpha)

    def rotZ(self, alpha)
	self.v[0][0]=cos(alpha)
	self.v[1][0]=sin(alpha)
	self.v[2][0]=0
	self.v[0][1]=-sin(alpha)
	self.v[1][1]=cos(alpha)
	self.v[2][1]=0
	self.v[0][2]=0
	self.v[1][2]=0
	self.v[2][2]=1

    def setAxisAngle(self, vec, alpha):
        auxvec = copy.deepcopy(vec)
        auxvec.normalize()

        c = cos(alpha)
        s = sin(alpha)
        t1 = 1-c
        t2 = auxvec[0]*t1

	self.v[0][0]=t2*auxvec[0] + c
	self.v[1][0]=t2*auxvec[1] + auxvec[2]*s
	self.v[2][0]=t2*auxvec[2] - auxvec[1]*s
	self.v[0][1]=t2*auxvec[1] - auxvec[2]*s
	self.v[1][1]=auxvec[1]*auxvec[1]*t1 + c
	self.v[2][1]=auxvec[1]*auxvec[2]*t1 + auxvec[0]*s
	self.v[0][2]=t2*auxvec[2] + auxvec[1]*s
	self.v[1][2]=auxvec[1]*auxvec[2]*t1 - auxvec[0]*s
	self.v[2][2]=auxvec[2]*auxvec[2]*t1 + c

    def getAngle(self):
        return acos((self.v[0][0]+self.v[1][1]+self.v[2][2]-1)/2)

    def getAxis(self):
        w = Vec(3)

        alpha = self.getAngle()
        s = sin(alpha)
        if (s != 0):
            t1 = 1/(2*s)
            w[0] = (self.v[2][1]-self.v[1][2])*t1
            w[1] = (self.v[0][2]-self.v[2][0])*t1
            w[2] = (self.v[1][0]-self.v[0][1])*t1
        else:
            if (alpha==0):
                w[0] = 0
                w[1] = 0
                w[2] = 0
            else:
                w[0] = sqrt((self.v[0][0]+1)/2)
                w[1] = self.v[1][2]/self.v[0][2]*w[0]
                w[2] = self.v[2][1]/self.v[0][1]*w[0]

        return w

    def getQuaternion(self):
        q = Quaternion()
        t0 = 1.0 + self.v[0][0] + self.v[1][1] + self.v[2][2]
        t1 = 1.0 + self.v[0][0] - self.v[1][1] - self.v[2][2]
        t2 = 1.0 - self.v[0][0] + self.v[1][1] - self.v[2][2]
        t3 = 1.0 - self.v[0][0] - self.v[1][1] + self.v[2][2]

        if ((t0 >= t1) and (t0 >= t2) and (t0 >= t3)):

            r = sqrt(t0)
            s = 0.5/r

            q[0] = 0.5*r
            q[1] = (self.v[2][1]-self.v[1][2])*s
            q[2] = (self.v[0][2]-self.v[2][0])*s
            q[3] = (self.v[1][0]-self.v[0][1])*s
        elif ((t1 >= t2) and (t1 >= t3)):
            r = sqrt(t1)
            s = 0.5/r

            q[0] = (self.v[2][1]-self.v[1][2])*s
            q[1] = 0.5*r
            q[2] = (selfv[0][1]+self.v[1][0])*s
            q[3] = (self.v[0][2]+self.v[2][0])*s
            
        elif (t2 >= t3):
            r = sqrt(t2)
            s = 0.5/r

            q[0] = (self.v[0][2] - self.v[2][0] ) * s
            q[1] = (self.v[0][1] + self.v[1][0] ) * s
            q[2] = 0.5 * r
            q[3] = (self.v[1][2] + self.v[2][1] ) * s
        else:
            r = sqrt(t3)
            s = 0.5/r

            q[0] = (self.v[1][0] - self.v[0][1]) * s
            q[1] = (self.v[0][2] + self.v[2][0] ) * s
            q[2] = (self.v[1][2] + self.v[2][1] ) * s
            q[3] = 0.5 * r

        return q


    def inv(self):
        return Mat.transp()
