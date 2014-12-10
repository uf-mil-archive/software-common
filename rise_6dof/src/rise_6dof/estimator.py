from __future__ import division

from numpy import *

from jacobi2d import jacobi2d

def estimator(zeta, nuC, nuCDot):
    eta = zeta[0:3];
    nu = zeta[3:6];

    u = nu[0];
    v = nu[1];
    r = nu[2];
    
    uc = nuC[0];
    vc = nuC[1];
    
    ur = nu[0] - nuC[0];
    vr = nu[1] - nuC[1];

    m = 40.9;
    Izz = 2.1;
    xg = 0;

    J = jacobi2d(eta[2])

    Mrb = zeros((3,3))
    Ma = zeros((3,3))
    Crb = zeros((3,3))

    Xud = -6.9638
    Yvd = -11.5454
    Yrd =  0.1529
    Nrd = -21.3437

    Mrb[0,0] =  m;
    Mrb[1,1] =  m;
    Mrb[1,2] =  m*xg;
    Mrb[2,1] =  m*xg;
    Mrb[2,2] =  Izz;

    Ma[0,0] =  -Xud;
    Ma[1,1] =  -Yvd;
    Ma[1,2] =  -Yrd;
    Ma[2,1] =  -Yrd;
    Ma[2,2] =  -Nrd;
    
    Mmat = Mrb + Ma;

    Minv = linalg.inv(Mmat)

    Crb[0,2] = -m*(xg*r + v)
    Crb[1,2] =  m*u
    Crb[2,0] =  m*(xg*r + v)
    Crb[2,1] = -m*u

    Y1 = array([
        [Minv[0,0]*ur*abs(ur), 0, 0],                                       # Xuu
        [0, Minv[1,1]*vr*abs(vr), Minv[2,1]*vr*abs(vr)],                      # Yvv
        [0, Minv[1,1]*r*abs(r), Minv[2,1]*r*abs(r)],                          # Yrr
        [0, Minv[1,2]*vr*abs(vr), Minv[2,2]*vr*abs(vr)],                      # Nvv
        [0, Minv[1,2]*r*abs(r), Minv[2,2]*r*abs(r)],                          # Nrr
        [-Minv[0,0]*r*vr, Minv[1,2]*ur*vr, Minv[2,2]*ur*vr],                  # Yvd
        [-Minv[0,0]*r**2, Minv[1,2]*r*ur, Minv[2,2]*r*ur],                     # Yrd
        [0, Minv[1,1]*r*ur-Minv[1,2]*ur*vr, Minv[2,1]*r*ur-Minv[2,2]*ur*vr],  # Xud
    ], dtype=float).transpose()

    Y2 = array([
        [Minv[0,0]*uc*abs(uc), 0, 0],
        [0, Minv[1,1]*vc*abs(vc), Minv[2,1]*vc*abs(vc)],
        [0, 0, 0],
        [0, Minv[1,2]*vc*abs(vc), Minv[2,2]*vc*abs(vc)],
        [0, 0, 0],
        [0, -Minv[1,2]*uc*vc, -Minv[2,2]*uc*vc],
        [0, 0, 0],
        [0, Minv[1,2]*uc*vc, Minv[2,2]*uc*vc],
    ], dtype=float).transpose()

    Y3 = array([
        [uc*abs(uc), 0, 0],
        [0, vc*abs(vc), 0],
        [0, 0, 0],
        [0, 0, vc*abs(vc)],
        [0, 0, 0],
        [0, 0, -uc*vc],
        [0, 0, 0],
        [0, 0, uc*vc],
    ], dtype=float).transpose()

    Y4 = array([
        [Minv[0,0]*u*abs(u), 0, 0],
        [0, Minv[1,1]*v*abs(v), Minv[2,1]*v*abs(v)],
        [0, Minv[1,1]*r*abs(r), Minv[2,1]*r*abs(r)],
        [0, Minv[1,2]*v*abs(v), Minv[2,2]*v*abs(v)],
        [0, Minv[1,2]*r*abs(r), Minv[2,2]*r*abs(r)],
        [-Minv[0,0]*r*v, Minv[1,2]*u*v, Minv[2,2]*u*v],
        [-Minv[0,0]*r**2, Minv[1,2]*r*u, Minv[2,2]*r*u],
        [0, Minv[1,1]*r*u-Minv[1,2]*u*v, Minv[2,1]*r*u-Minv[2,2]*u*v],
    ], dtype=float).transpose()


    Y1 = vstack([
        zeros((3,8)),
        Y1,
    ])
    Y2 = vstack([
        zeros((3,8)),
        Y2,
    ])
    
    Y4 = vstack([
        zeros((3,8)),
        Y4,
    ])

    f0 = vstack([
        J.dot(nu),
        -Minv.dot(Crb).dot(nu),
    ])
    
    f1 = vstack([
        zeros((3,1)),
        Minv.dot(Ma).dot(nuCDot),
    ])

    g = vstack([
        zeros((3,3)),
        Minv,
    ])

    return Y1, Y2, Y3, Y4, f0, f1, g

if __name__ == '__main__':
    set_printoptions(linewidth=100000)
    for x in estimator(array([[1,2,3,4,5,6]]).transpose(), array([[7,8,9]]).transpose(), array([[10,11,12]]).transpose()):
        print x
        print
