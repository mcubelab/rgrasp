#!/usr/bin/python


from ik.ik import *

print 'Call fastik_python 1000 times'
with Timer('fastik_python'):
    for i in xrange(1000):
        x= fastik_python(targetpose = [0.531, -0.001, 1.201, -0.0044562, 0.70843, -0.0012583, 0.70576], q0 = [0,0,0,0,0,0])

print 'solution', x

print ''

print 'Call fastik_python 1000 times'
with Timer('fastik'):
    for i in xrange(1000):
        fastik(targetpose = [0.531, -0.001, 1.201, -0.0044562, 0.70843, -0.0012583, 0.70576], q0 = [0,0,0,0,0,0])

print 'solution', x
