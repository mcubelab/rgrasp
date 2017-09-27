from numpy import *
import Vec.py
import Mat.py
import RotMat.py


class HomogTransf(Mat):

    def __init__(self):
        Mat.__init__(0.0,4,4)
        for i in range(4):
            self.v[i][i] = 1.0

    def __init__(self, orig):
        Mat.__init__(orig)

    def __init__(self, vals):
        Mat.__init__(vals, 4,4)

    def __init__(self, vals_str):
        Mat.__init__(vals_str,4,4)

    def __init__(self, rot, trans):
        Mat.__init__(4,4)
        self.setRotation(rot)
        self.setTranslation(trans)
        self.v[3][0] = 0
        self.v[3][1] = 0
        self.v[3][2] = 0
        self.v[3][3] = 1

    def __init__(self, orig):
        Mat.__init__(4,4)
        if ((orig.nn==4) and (orig.mm==4)):
            for i in range(4):
                for j in range(4):
                    v[i][j] = orig[i][j]

# Eplicitly inherited for preserving the output class label
    def  __iadd__(self, orig):
        self = HomogTransf(self+=orig)
        return self

    def  __isub__(self, orig):
        self = HomogTransf(self-=orig)
        return self

    def  __imul__(self, const):
        self = HomogTransf(self*=const)
        return self
    
    def  __idiv__(self, const):
        self = HomogTransf(self/=const)
        return self

    def __add__(self, orig):
        return HomogTransf(self+orig)

    def __sub__(self, orig):
        return HomogTransf(self-orig)

    def __mul__(self, orig): # HomogTransf,Vec,double orig
        return HomogTransf(self*orig)

    def __div__(self, const):
        return HomogTransf(self/const)

    def __mul__(self, orig): #orig is Vec
        if (orig.nn == 3):
            w = Vec(0.0,3)
            if (orig.nn == 3):
                for i in range(3):
                    for j in range(3):
                        w[i] += self.v[i][j]*orig[j]
                        w[i] += self.v[i][3]
            return w
        else:
            return Mat(self*orig)

    def setRotation(self, rot):
        for i in range(3):
            for j in range(3):
                self.v[i][j] = rot[i][j]

    def setTranslation(self, trans):
        self.v[0][3] = trans[0]
        self.v[1][3] = trans[1]
        self.v[2][3] = trans[2]

    def getRotation(self):
        r = RotMat()
        for i in range(3):
            for j in range(3):
                r[i][j] = self.v[i][j]
        return r

    def getTranslation(self):
        trans = Vec(3)
        trans[0] = self.v[0][3] 
        trans[1] = self.v[1][3]
        trans[2] = self.v[2][3]
        return trans

    def inv(self):
        h = HomogTransf()
        h[0][0] = self.v[0][0]
        h[0][1] = self.v[1][0]	
        h[0][2] = self.v[2][0]
	h[1][0] = self.v[0][1]	
        h[1][1] = self.v[1][1]	
        h[1][2] = self.v[2][1] 
	h[2][0] = self.v[0][2]	
        h[2][1] = self.v[1][2]	
        h[2][2] = self.v[2][2] 

	h[0][3] =  - (self.v[0][3]*self.v[0][0]) - (self.v[1][3]*self.v[1][0]) - (self.v[2][3]*self.v[2][0])
	h[1][3] =  - (self.v[0][3]*self.v[0][1]) - (self.v[1][3]*self.v[1][1]) - (self.v[2][3]*self.v[2][1])
	h[2][3] =  - (self.v[0][3]*self.v[0][2]) - (self.v[1][3]*self.v[1][2]) - (self.v[2][3]*self.v[2][2])

	h[3][0] = 0.0
        h[3][1] = 0.0
        h[3][2] = 0.0
        h[3][3] = 1.0

	return h










