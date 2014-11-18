from __future__ import division

from numpy import *

from jacobi2d import jacobi2d

def estimator(x):
    eta = zeros((3, 1))
    eta[0,0] = x[0] # state
    eta[1,0] = x[1]
    eta[2,0] = x[2]
    nu = zeros((3, 1))
    nu[0,0] = x[3] # \dot{state}
    nu[1,0] = x[4]
    nu[2,0] = x[5]

    u = nu[0]
    v = nu[1]
    r = nu[2]

    m = 40.9
    Izz = 2.1
    xg = 0

    J = jacobi2d(eta[2])

    Mmat = zeros((3,3))
    Crb = zeros((3,3))

    # Yvv = -5.3346e+03
    # Yrr =  7.7716
    # Yur =  34.7555
    # Nvv = -0.1387
    # Nrr = -43.6724
    # Nuv = -602.9014
    # Xuu = -57.5229
    Xud = -6.9638
    Yvd = -11.5454
    Yrd =  0.1529
    Nrd = -21.3437

    Mmat[0,0] =  m - Xud
    Mmat[1,1] =  m - Yvd
    Mmat[1,2] =  m*xg - Yrd
    Mmat[2,1] =  m*xg - Yrd
    Mmat[2,2] =  Izz-Nrd

    Minv = linalg.inv(Mmat)

    Crb[0,2] = -m*(xg*r + v)
    Crb[1,2] =  m*u
    Crb[2,0] =  m*(xg*r + v)
    Crb[2,1] = -m*u

    '''Y = array([
        [Minv[0,0]*u*abs(u), 0, 0],                        # Xuu
        [0, Minv[1,1]*v*abs(v), Minv[2,1]*v*abs(v)],        # Yvv
        [0, Minv[1,1]*r*abs(r), Minv[2,1]*r*abs(r)],        # Yrr
        [0, Minv[1,2]*v*abs(v), Minv[2,2]*v*abs(v)],        # Nvv
        [0, Minv[1,2]*r*abs(r), Minv[2,2]*r*abs(r)],        # Nrr
        [-Minv[0,0]*r*v, Minv[1,2]*u*v, Minv[2,2]*u*v],     # Yvd
        [-Minv[0,0]*r**2, Minv[1,2]*r*u, Minv[2,2]*r*u],     # Yrd
        [0, Minv[1,1]*r*u, Minv[2,1]*r*u],                   # Yur
        [0, Minv[1,2]*u*v, Minv[2,2]*u*v],                 # Nuv
    ], dtype=float).transpose()'''
    
    Y = array([[Minv[0,0]*u*abs(u), 0, 0],                                        # Xuu
        [0, Minv[1,1]*v*abs(v), Minv[2,1]*v*abs(v)],                        # Yvv
        [0, Minv[1,1]*r*abs(r), Minv[2,1]*r*abs(r)],                        # Yrr
        [0, Minv[1,2]*v*abs(v), Minv[2,2]*v*abs(v)],                        # Nvv
        [0, Minv[1,2]*r*abs(r), Minv[2,2]*r*abs(r)],                        # Nrr
        [-Minv[0,0]*r*v, Minv[1,2]*u*v, Minv[2,2]*u*v],                     # Yvd
        [-Minv[0,0]*r**2, Minv[1,2]*r*u, Minv[2,2]*r*u],                    # Yrd
        [0, Minv[1,1]*r*u-Minv[1,2]*u*v, Minv[2,1]*r*u-Minv[2,2]*u*v]], dtype=float).transpose()    # Xud

    Y = vstack([
        zeros((3,8)),
        Y,
    ])

    f0 = vstack([
        J.dot(nu),
        -Minv.dot(Crb).dot(nu),
    ])

    g = vstack([
        zeros((3,3)),
        Minv,
    ])

    return Y, f0, g

if __name__ == '__main__':
    for x in estimator([1,2,3,4,5,6]):
        print x
        print
