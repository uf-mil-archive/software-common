from __future__ import division

import time

import numpy

from rise_6dof import reducedStationMain


radp = reducedStationMain.RADPController()

s = time.time()
c = 0
while time.time() < s + 5:
    radp.step(1/50, numpy.array([[1,2,3,4,5,6]]).transpose())
    c += 1
e = time.time()

print (e-s)/c/1e-3
