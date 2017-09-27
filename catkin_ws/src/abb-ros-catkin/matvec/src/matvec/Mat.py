from math import *

from Vec.py import *


class Mat:
    
    def __init__(self):
        self.nn = 0
        self.mm = 0
        self.v = 0

    def __init__(self, n, m):
        if ((n>0) and (m>0)):
            self.nn = n
            self.mm = m
            self.v = [[0 for x in xrange(m)] for x in xrange(n)] 
        else:
            Mat()

    def __init__(self, const, n, m):
        if ((n>0) and (m>0)):
            self.nn = n
            self.mm = m
            self.v = [[const for x in xrange(m)] for x in xrange(n)] 
        else:
            Mat()

    def __init__(self, vals, n, m):
        if ((n>0) and (m>0)):
            self.nn = n
            self.mm = m
            k = 0
            for i in range(n):
                for j in range(m):
                    self.v[i][j] = vals[k]
                    k += 1
        else:
            Mat()

    def __init__(self, vals_str, n, m):
        if ((n<0) and (m>0)):
            self.nn = n
            self.mm = m
            vals = [float(i) for i in vals_str.split(" ")]
            k = 0
            for i in range(n):
                for j in range(m):
                    self.v[i][j] = vals[k]
                    k += 1
        else:
            Mat()

    def __init__(self, mat):
        self = copy.deepcopy(mat)

    def __del__(self):
        if (self.v != 0):
            del(self.v)

    def __iadd__(self, other):
        if (self.nn == other.nn) and (self.mm == other.mm):
            self.v = [self.v[i][j]+other.v[i][j] \
                          for i in self.nn for j in self.mm]
        return self

    def __isub__(self, other):
        if (self.nn == other.nn) and (self.mm == other.mm):
            self.v = [self.v[i][j]-other.v[i][j] \
                          for i in self.nn for j in self.mm]
        return self        

    def __iadd__(self, const):
        self.v = [self.v[i][j]+const for i in self.nn for j in self.mm]
        return self

    def __isub__(self, const):
        self.v = [self.v[i][j]-const for i in self.nn for j in self.mm]
        return self        

    def __imul__(self, const):
        self.v = [self.v[i][j]*const for i in self.nn for j in self.mm]
        return self

    def __idiv__(self, const):
        self.v = [self.v[i][j]/const for i in self.nn for j in self.mm]
        return self        

    def __add__(self, other):
        w = Mat(self)
        w += other
        return w

    def __sub__(self, other):
        w = Mat(self)
        w -= other
        return w

    def __neg__(self):
        w = Mat(self.nn, self.mm)
        for i in range(w.nn):
            for j in range(w.mm):
                w[i][j] = -self.v[i][j]
        return w

    def __mul__(self, other):
        w = Vec()
        if (self.mm == other.nn):
            w = Vec(0.0, self.nn)
            for i in range(self.nn):
                for j in range(self.mm):
                    w[i] += self.v[i][j] * other[j]
        return w

    def __add__(self, const):
        w = Mat(self)
        return w += const

    def __sub__(self, const):
        w = Mat(self)
        return w -= const

    def __mul__(self, const):
        w = Mat(self)
        return w *= const

    def __div__(self, const):
        w = Mat(self)
        return w /= const

    def __str__(self):
        ans = "["
        for i in range(self.nn):
            for j in range(self.mm):
                ans += str(self.v[i][j])+" "
        ans += "]"
        return ans

    def LDU(self, perms, sign):
        print "LDU NOT IMPLEMENTED"
        pass

    def LDUsolve(self, perms, sign):
        print "LDU NOT IMPLEMENTED"
        pass

    def LDUdet(self, perms, sign):
        print "LDU NOT IMPLEMENTED"
        pass

    def transp(self):
        w = Mat(self.mm, self.nn)
        for i in range(self.nn):
            for j in range(self.mm):
                w[j][i] = self.v[i][j]
        return w

    def det(self):
        print "DETERMINATE NOT IMPLEMENTED"
        pass

    def getRow(self, n):
        r = Vec(self.mm)
        if ((n>=0) and (n<self.nn)):
            for i in range(self.mm):
                r[i] = self.v[n][i]
        return r    

    def getCol(self, m):
        c = Vec(self.nn)
        if ((m>=0) and (m<self.mm)):
            for i in range(self.nn):
                c[i] = self.v[i][m]
        return c

    def setRow(self, n, row):
        if ((n>=0) and (n<self.nn) and (row.nn==self.mm)):
            for i in range(self.mm):
                self.v[n][i] = row[i]

    def setCol(self, m, col):
        if ((m>=0) and (m<self.mm) and (col.nn==nn)):
            for i in range(self.nn):
                self.v[i][m] = col[i]
            
    def LSsolve(self):
        print "LSsolve NOT IMPLEMENTED"
        pass

    def mean(self):
        aux = sum([sum(row) for row in self.v])
        return (aux)/float(self.nn*self.mm)

    def variance(self):
        if ((self.nn*self.mm) <= 1):
            return 0
        else:
            return np.var(self.v)

    def stdev(self):
        return sqrt(self.variance())

















