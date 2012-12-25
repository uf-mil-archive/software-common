from __future__ import division

import numpy

from tf import transformations


normalized = lambda x: x/numpy.linalg.norm(x)

def triad((a1, a2), (b1, b2)):
    # returns quaternion that rotates b1 to a1 and b2 to a2
    # can get orientation by passing in (global, local)
    aa = normalized(numpy.cross(a1, a2))
    A = numpy.array([normalized(a1), aa, normalized(numpy.cross(a1, aa))])
    bb = normalized(numpy.cross(b1, b2))
    B = numpy.array([normalized(b1), bb, normalized(numpy.cross(b1, bb))])
    rot = A.T.dot(B)
    return transformations.quaternion_from_matrix([(a,b,c,0) for a,b,c in rot]+[(0,0,0,1)])

def test_triad():
    q = transformations.random_quaternion()

    import random
    a = numpy.array([random.gauss(0, 1) for i in xrange(3)])
    b = numpy.array([random.gauss(0, 1) for i in xrange(3)])

    m = transformations.quaternion_matrix(q)[:3, :3]
    q_ = triad((m.dot(a), m.dot(b)), (a, b))

    print list(q)
    print list(q_)
