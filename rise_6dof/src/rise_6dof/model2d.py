from __future__ import division

from numpy import *
from numpy.linalg import inv

from jacobi2d import jacobi2d

def model2d(zeta):
    #REMUS Equations of motion for REMUS AUV in body-fixed coordinates

    # -----------------------------------------------------------------
    # STATE VECTOR:
    #  Earth-fixed coordinates
    #  xp     = Position in x-direction   [m]
    #  yp     = Position in y-direction   [m]
    #  psi    = Yaw angle                 [rad]
    # -----------------------------------------------------------------

    eta = zeta[0:3];
    nu = zeta[3:6];

    m = 40.9;           # kg
    Izz = 2.1;          # kg

    xg = 0;             # m

    # Yvv     = -1310.0;  # kg/m
    # Yrr     =  0.632;   # kg.m/rad^2
    # Yur     =  5.22;    # kg/rad
    # Nvv     = -3.18;    # kg
    # Nrr     = -94.0;    # kg.m/rad^2
    # Nuv     = -24.0;    # kg
    # Xuu     = -2.9355;  # kg/m
    # Xud     = -0.93;    # kg
    # Yvd     = -35.5;    # kg
    # Yrd     =  1.93;    # kg.m/rad
    # Nrd     = -4.88;    # kg.m^2/rad

    # Identified Data
    Yvv = -5.3346e+03;
    Yrr =  7.7716;
    Yur =  34.7555;
    Nvv = -0.1387;
    Nrr = -43.6724;
    Nuv = -602.9014;
    Xuu = -57.5229;
    Xud = -6.9638;
    Yvd = -11.5454;
    Yrd =  0.1529;
    Nrd = -21.3437;

    u = nu[0];
    v = nu[1];
    r = nu[2];

    Mmat = zeros([3,3]);
    Crb = zeros([3,3]);
    Ca = zeros([3,3]);
    D = zeros([3,3]);
    g = zeros([3,1]);

    Mmat[0,0] =  m - Xud;
    Mmat[1,1] =  m - Yvd;
    Mmat[1,2] =  m*xg - Yrd;
    Mmat[2,1] =  m*xg - Yrd;
    Mmat[2,2] =  Izz-Nrd;

    Crb[0,2] = -m*(xg*r + v);
    Crb[1,2] =  m*u;
    Crb[2,0] =  m*(xg*r + v);
    Crb[2,1] = -m*u;

    Ca[0,2] =  Yvd*v + Yrd*r;
    Ca[1,2] = -Yur*u;
    Ca[2,0] = -Yvd*v - Yrd*r;
    Ca[2,1] = -Nuv*u;

    C = Crb + Ca;

    D[0,0] = -Xuu*abs(u);
    D[1,1] = -Yvv*abs(v);
    D[1,2] = -Yrr*abs(r);
    D[2,1] = -Nvv*abs(v);
    D[2,2] = -Nrr*abs(r);

    g[0] = 0;
    g[1] = 0;
    g[2] = 0;

    J = jacobi2d(eta[2]);

    f = concatenate([J.dot(nu), -inv(Mmat).dot(C).dot(nu) - inv(Mmat).dot(D).dot(nu)]);
    g = concatenate([zeros([3,3]), inv(Mmat)]);

    # f = [zeta[3]; zeta[4]; zeta[5]; 0; 0; 0];
    # g = [zeros([3,3]); eye(3)];

    return f, g
