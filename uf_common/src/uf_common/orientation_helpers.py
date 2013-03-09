from __future__ import division

import math
import warnings

import numpy

import roslib
roslib.load_manifest('uf_common')
from tf import transformations


def normalized(x):
    x = numpy.array(x)
    if max(map(abs, x)) == 0:
        warnings.warn('Normalizing zero-length vector to random unit vector')
        x = numpy.random.standard_normal(x.shape)
    x = x / max(map(abs, x))
    x = x / numpy.linalg.norm(x)
    return x

def get_perpendicular(a, b=None):
    a = numpy.array(a)
    if max(map(abs, a)) == 0:
        if b is not None: return get_perpendicular(b)
        return normalized(numpy.random.standard_normal(a.shape))
    if b is None:
        b = numpy.random.standard_normal(a.shape)
    b = numpy.array(b)
    x = numpy.cross(a, b)
    if max(map(abs, x)) == 0:
        return get_perpendicular(a)
    return normalized(x)


def quat_to_rotvec(q):
    if q[3] < 0:
        q = -q
    q = transformations.unit_vector(q)
    angle = math.acos(q[3])*2
    axis = normalized(q[0:3])
    return axis * angle

def rotvec_to_quat(rotvec):
    return transformations.quaternion_about_axis(numpy.linalg.norm(rotvec), rotvec)


xyz_array = lambda o: numpy.array([o.x, o.y, o.z])
xyzw_array = lambda o: numpy.array([o.x, o.y, o.z, o.w])


def triad((a1, a2), (b1, b2)):
    # returns quaternion that rotates b1 to a1 and b2 near a2
    # can get orientation by passing in (global, local)
    aa = get_perpendicular(a1, a2)
    A = numpy.array([normalized(a1), aa, normalized(numpy.cross(a1, aa))])
    bb = get_perpendicular(b1, b2)
    B = numpy.array([normalized(b1), bb, normalized(numpy.cross(b1, bb))])
    rot = A.T.dot(B)
    return transformations.quaternion_from_matrix(
        [(a,b,c,0) for a,b,c in rot]+
        [(0,0,0,1)])

def test_triad():
    q = transformations.random_quaternion()
    
    a = numpy.random.standard_normal(3)
    b = numpy.random.standard_normal(3)
    
    m = transformations.quaternion_matrix(q)[:3, :3]
    q_ = triad((m.dot(a), m.dot(b)), (a, b))
    
    assert numpy.linalg.norm(quat_to_rotvec(
        transformations.quaternion_multiply(
            q,
            transformations.quaternion_inverse(q_),
        )
    )) < 1e-6
test_triad()


def lookat(forward, upish=[0, 0, 1]):
    # assumes standard forward-left-up body coordinate system
    return triad((forward, upish), ([1, 0, 0], [0, 0, 1]))
def lookat_camera(forward, upish=[0, 0, 1]):
    # assumes camera right-down-forward coordinate system
    return triad((forward, upish), ([0, 0, 1], [0, -1, 0]))
