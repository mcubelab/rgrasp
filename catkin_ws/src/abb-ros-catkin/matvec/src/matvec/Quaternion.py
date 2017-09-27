from math import *
import random
import copy

import Vec.py

class Quaternion:

    def _init__(self):
        self.v = Vec(4)
        
    def __init__(self, const):
        self.v = Vec(const, 4)

    def __init__(self, vals):
        self.v = Vec(vals, 4)

    def __init__(self, vals_str):
        self.v = Vec(vals_str, 4)
        
    def __init__(self, q0, qv):
        self.v[0] = q0
        self.v[1] = qv[0]
        self.v[2] = qv[1]
        self.v[w] = qv[2]

    def __init__(self, orig_vec):
        if (orig_vec.nn == 4):
            self.v = copy.deepcopy(orig_vec)
            
    def __init__(self, quat):
        self = copy.deepcopy(quat)

    def __getitem__(self, i):
        if (i >= 0) and (i <= 4):
            return self.v[i]
        else:
            return 0.0

    def __iadd__(sef, orig):
        self = Quaternion(self.v+=orig.v)

    def __isub__(self, orig):
        self = Quaternion(self.v-=orig.v)
        
    def __add__(self, orig):
        return Quaternion(self.v+orig.v)

    def __sub__(self, orig):
        return Quaternion(self.v-orig.v)

    def __neg__(self):
        return Quaternion(-self.v)

    def __mul__(self, const):
        return Quaternion(self.v*const)
    
    def __div__(self, const):
        return Quaternion(self.v/const)

    def __pow__(self, orig):
        w = Quaternion()
        w[0]=self[0]*orig[0] - self[1]*orig[1] - \
            self[2]*orig[2] - self[3]*orig[3]
	w[1]=self[0]*orig[1] + self[1]*orig[0] + \
            self[2]*orig[3] - self[3]*orig[2]
	w[2]=self[0]*orig[2] - self[1]*orig[3] + \
            self[2]*orig[0] + self[3]*orig[1]
	w[3]=self[0]*orig[3] + self[1]*orig[2] - \
            self[2]*orig[1] + self[3]*orig[0]

	return w

    def __mul__(self, orig): 
        return sum([self[i]*orig[i] for i in range(4)])

    def conjugate(self):
        w = Quaternion()
        w[0] = self[0]
        w[1] = -self[1]
        w[2] = -self[2]
        w[3] = -self[3]
        return w

    def getScalar(self):
        return self[0]

    def setScalar(self, s):
        self[0] = s

    def getVector(self):
        w = Vec(3)
        w[0] = self[1]
        w[1] = self[2]
        w[2] = self[3]
        return w

    def setVector(self, vec):
        if (vec.nn == 3):
            self[1] = vec[0]
            self[2] = vec[1]
            self[3] = vec[2] 
            
    def inverse(self):
        n = self.v.norm()
        if (n != 0):
            return self.conjugate()/(n*n)
        else:
            return Quaternion(0.0)

    def leftMat(self):
        w = Mat(0.0,4,4)
	w[0][0]=self[0]   
        w[0][1]=-self[1]   
        w[0][2]=-self[2]   
        w[0][3]=-self[3]
  
	w[1][0]=self[1]   
        w[1][1]=self[0]   
        w[1][2]=-self[3]   
        w[1][3]=self[2]   

	w[2][0]=self[2]   
        w[2][1]=self[3]   
        w[2][2]=self[0]   
        w[2][3]=-self[1]   

	w[3][0]=self[3]   
        w[3][1]=-self[2]   
        w[3][2]=self[1]   
        w[3][3]=self[0]   

	return w        

    def rightMat(self):
        w = Mat(0.0,4,4)
	w[0][0]=self[0]   
        w[0][1]=-self[1]   
        w[0][2]=-self[2]   
        w[0][3]=-self[3]
  
	w[1][0]=self[1]  
        w[1][1]=self[0]   
        w[1][2]=self[3]   
        w[1][3]=-self[2] 
        
	w[2][0]=self[2]   
        w[2][1]=-self[3]   
        w[2][2]=self[0]   
        w[2][3]=self[1] 
  
	w[3][0]=self[3]   
        w[3][1]=self[2]   
        w[3][2]=-self[1]   
        w[3][3]=self[0]   

	return w

    def dist(self, orig):
        d1 = (self - orig).norm()
        d2 = (self + orig).norm()
        if (d1 <= d2):
            return d1
        return d2

    def getAngle(self):
        vec = self.getVector()
        s = self.getScalar()
        a = 2*atan2(vec.norm(), s)
        if (a > pi):
            a = 2*pi - a
        return a

    def getAxis(self):
        n = self.getVector().norm()
        if (n != 0):
            return self.getVector()/n
        else:
            return Vec("0.0 0.0 0.0",3)

    def getRotMat(self):
        w = RotMat()
	w[0][0]=self[0]*self[0]+self[1]*self[1]-self[2]*self[2]-self[3]*self[3];
	w[0][1]=2*(self[1]*self[2]-self[0]*self[3])
	w[0][2]=2*(self[1]*self[3]+self[0]*self[2])
	w[1][0]=2*(self[1]*self[2]+self[0]*self[3])
	w[1][1]=self[0]*self[0]-self[1]*self[1]+self[2]*self[2]-self[3]*self[3]
	w[1][2]=2*(self[2]*self[3]-self[0]*self[1])
	w[2][0]=2*(self[1]*self[3]-self[0]*self[2])
	w[2][1]=2*(self[2]*self[3]+self[0]*self[1])
	w[2][2]=self[0]*self[0]-self[1]*self[1]-self[2]*self[2]+self[3]*self[3]
	return w























