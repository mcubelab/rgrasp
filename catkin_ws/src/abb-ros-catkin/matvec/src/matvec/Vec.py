# TODO dependence on Mat?
import numpy as np
import deepcopy
import random

class Vec:

    def __init__(self):
        self.nn = 0
        self.v = 0

    def __init__(self, n):
        if (n >= 0):
            self.nn = n
            self.v = [0 for i in range(n)] # size n 
        else:
            self.nn = 0
            self.v = 0

    def __init__(self, const, n):
        if (n >= 0):
            self.nn = n
            self.v = [const for i in range(n)] # size n
        else:
            self.nn = 0
            self.v = 0

    def __init__(self, vals, n):
        if (n >= 0):
            self.nn = n
            self.v = copy.deepcopy(vals)
	else:
            self.nn = 0
            self.v = 0

    def __init__(self, vals_str, n):
        if (n > 0):
            self.nn = n
            if len(vals_str.split(" ")) != nn:
                print "Inconsistent number of vector elements with size"
            self.v = [float(i) for i in vals_str.split(" ")]
        else:
            self.nn = 0
            self.v = 0

    def __init__(self, vec):
        self.nn = copy.deepcopy(vec.nn)
        self.v = copy.deepcopy(vec.v)

    def __getitem__(self, i):
        if (i >= 0) and (i < self.nn): 
            return self.v[i]
        else:
            return 0.0

    def __iadd__(self, orig):
        if (self != orig):
            if (self.nn == orig.nn):
                self.v = [self[i]+orig[i] for i in range(self.nn)]
        return self
    
    def __isub__(self, orig):
        if (self != orig):
            if (self.nn == orig.nn):
                self.v = [self[i]-orig[i] for i in range(self.nn)]
        return self

    def __imul__(self, const):
        self.v = [self[i]*const for i in range(self.nn)]
        return self        

    def __idiv__(self, const):
        self.v = [self[i]/cons for i in range(self.nn)]
        return self        

    def __add__(self, orig):
        if (self.nn == orig.nn):
            w = Vec(self.nn)
            w.v = [self[i]+orig[i] for i in range(self.nn)]
            return w
        return Vec(0)

    def __sub__(self, orig):
        if (self.nn == orig.nn):
            w = Vec(self.nn)
            w.v = [self[i]-orig[i] for i in range(self.nn)]
            return w
        return Vec(0)

    def __neg__(self):
        w = Vec(nn)
        w.v = [-x for x in self]
        return w

    def __mul__(self, const):
        w = Vec(nn)
        w = [const*x for x in self]
        return w

    def __div__(self, const):
        if (const != 0):
            w = Vec(nn)
            w.v = [x/const for x in self]
            return w
        w = Vec(0)
        return w

    def __add__(self, const):
        w = Vec(nn)
        w.v = [const+x for x in self]
        return w

    def __sub__(self, const):
        w = Vec(nn)
        w.v = [x-const for x in self]
        return w

    def __mul__(self, orig): # dot prod
        a = 0
        if (self.nn == orig.nn):
            a += sum([self[i]*orig[i] for i in range(self.nn)])
        return a

    def __pow__(self, orig):
        w = Vec(0.0,3)
        if (self.nn == 3) and (orig.nn == 3):
            w[0] = self[1]*orig[2] - self[2]*orig[1]
            w[1] = self[2]*orig[0] - self[0]*orig[2]
            w[2] = self[0]*orig[1] - self[1]*orig[0]
        return w

    def __str__(self):
        ans = "["
        for i in self:
            ans += str(i)+" "
        ans += "]" 
        return ans

    def norm(self):
        w = copy.deepcopy(self)
        return sqrt(w*w)

    def normalize(self):
        n = self.norm()
        for i in self.v:
            i /= n
            
    def max(self):
        return max(self.v)
    
    def min(self):
        return min(self.v)

    def maxInd(self):
        return np.argmax(self.v)

    def minInd(self):
        return np.argmin(self.v)

    def mean(self):
        return sum(self.v)/float(self.nn)

    def variance(self):
        if (self.nn <= 1):
            return 0
        else:
            m = self.mean()
            aux = sum([(i-m)*(i-m) for i in self])
            return aux/float(self.nn-1)

    def stdev(self):
        return sqrt(self.variance())

    def randPerm(self):
        random.shuffle(self.v)
        
    def abs(self):
        w = Vec(self.nn)
        w.v = [abs(i) for i in self]
        return w

    def __del__(self):
        if (self.v != 0):
            del(self.v)
