import math

import numpy

from tf import transformations

skewmatrix = lambda (x, y, z): numpy.array([[0,-z,y],
                                            [z,0,-x],
                                            [-y,x,0]])

rotvec_to_quat = lambda (x, y, z): transformations.quaternion_about_axis(numpy.linalg.norm((x, y, z)), (x, y, z))

def quat_to_rotvec(q):
    if q[3] < 0:
        q = -q
    q = transformations.unit_vector(q)
    angle = math.acos(q[3])*2
    axis = q[0:3]/numpy.linalg.norm(q[0:3]) if numpy.linalg.norm(q[0:3]) else numpy.zeros(3)
    return axis * angle
