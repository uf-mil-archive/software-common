from __future__ import division

from numpy import *
from numpy.linalg import inv

from jacobi2d import jacobi2d

def model2d(zeta, nuC, nuCDot, tau):
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
    nuR = nu - nuC
    m = 40.9;           # kg
    Izz = 2.1;          # kg

    xg = 0;             # m

    # Identified Data
    Yvv = -5.3346e+03;
    Yrr =  7.7716;
    Nvv = -0.1387;
    Nrr = -43.6724;
    Xuu = -57.5229;
    Xud = -6.9638;
    Yvd = -11.5454;
    Yrd =  0.1529;
    Nrd = -21.3437;

    u = nu[0];
    v = nu[1];
    r = nu[2];

    Mrb = zeros([3,3]);
    Ma = zeros([3,3]);
    Crb = zeros([3,3]);

    Mrb[0,0] =  m;
    Mrb[1,1] =  m;
    Mrb[1,2] =  m*xg;
    Mrb[2,1] =  m*xg;
    Mrb[2,2] =  Izz;

    Ma[0,0] = -Xud;
    Ma[1,1] = -Yvd;
    Ma[1,2] = -Yrd;
    Ma[2,1] = -Yrd;
    Ma[2,2] = -Nrd;

    Crb[0,2] = -m*(xg*r + v);
    Crb[1,2] =  m*u;
    Crb[2,0] =  m*(xg*r + v);
    Crb[2,1] = -m*u;

    Ca = lambda nu: array([
        [0, 0, Yvd*nu[1]+Yrd*nu[2]],
        [0, 0, -Xud*nu[0]],
        [-Yvd*nu[1]-Yrd*nu[2], Xud*nu[0], 0],
    ], dtype=float);

    D = lambda nu: array([
        [-Xuu*abs(nu[0]), 0, 0],
        [0, -Yvv*abs(nu[1]), -Yrr*abs(nu[2])],
        [0, -Nvv*abs(nu[1]), -Nrr*abs(nu[2])],
    ], dtype=float);

    Mmat = Mrb + Ma;

    J = jacobi2d(eta[2]);

    f = concatenate([J.dot(nu), inv(Mmat).dot(Ma).dot(nuCDot) - inv(Mmat).dot(Crb).dot(nu) - inv(Mmat).dot(Ca(nuR)).dot(nuR) - inv(Mmat).dot(D(nuR)).dot(nuR)]);
    g = concatenate([zeros([3,3]), inv(Mmat)]);
    
    xDot = f + g.dot(tau);

    # f = [zeta[3]; zeta[4]; zeta[5]; 0; 0; 0];
    # g = [zeros([3,3]); eye(3)];

    # Ca(1,3) =  Yvd*v + Yrd*r;
    # Ca(2,3) = -Yur*u;
    # Ca(3,1) = -Yvd*v - Yrd*r;
    # Ca(3,2) = -Nuv*u;
     
    # D(1,1) = -Xuu*abs(u);
    # D(2,2) = -Yvv*abs(v);
    # D(2,3) = -Yrr*abs(r);
    # D(3,2) = -Nvv*abs(v);
    # D(3,3) = -Nrr*abs(r);
    
    # Yvv     = -1310.0;  % kg/m
    # Yrr     =  0.632;   % kg.m/rad^2
    # Yur     =  5.22;    % kg/rad
    # Nvv     = -3.18;    % kg
    # Nrr     = -94.0;    % kg.m/rad^2
    # Nuv     = -24.0;    % kg
    # Xuu     = -2.9355;  % kg/m
    # Xud     = -0.93;    % kg
    # Yvd     = -35.5;    % kg
    # Yrd     =  1.93;    % kg.m/rad
    # Nrd     = -4.88;    % kg.m^2/rad

    return xDot

if __name__ == '__main__':
    print model2d(array([[1,2,3,4,5,6]]).transpose(), array([[7,8,9]]).transpose(), array([[10,11,12]]).transpose(), array([[13,14,15]]).transpose())
