from __future__ import division

from numpy import *

def jacobi2d(psi):
    
    J1 = zeros([3, 3]);

    J1[0,0] =  cos(psi);
    J1[0,1] = -sin(psi);
    J1[1,0] =  sin(psi);
    J1[1,1] =  cos(psi);
    J1[2,2] =  1;    

    return J1
