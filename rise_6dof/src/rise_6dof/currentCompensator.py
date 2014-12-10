from __future__ import division

from numpy import *


def currentCompensator(theta, nu, nuC, nuCDot):
    Xuda = -6.9638;
    Yvda = -11.5454;
    Yrda =  0.1529;
    Nrda = -21.3437;

    Xuu = theta[0];
    Yvv = theta[1];
    Yrr = theta[2];
    Nvv = theta[3];
    Nrr = theta[4];
    Yvd = theta[5];
    Yrd = theta[6];
    Xud = theta[7];

    # Yvv = -5.3346e+03;
    # Yrr =  7.7716;
    # Nvv = -0.1387;
    # Nrr = -43.6724;
    # Xuu = -57.5229;
    # Xud = -6.9638;
    # Yvd = -11.5454;
    # Yrd =  0.1529;
    # Nrd = -21.3437;

    Ma = zeros((3, 3))
    Ma[0,0] =  -Xuda;
    Ma[1,1] =  -Yvda;
    Ma[1,2] =  -Yrda;
    Ma[2,1] =  -Yrda;
    Ma[2,2] =  -Nrda;

    nuR = nu - nuC;

    Ca = lambda nu: array([ [0, 0, Yvd*nu[1]+Yrd*nu[2]],
        [0, 0, -Xud*nu[0]],
        [-Yvd*nu[1]-Yrd*nu[2], Xud*nu[0], 0]], dtype=float);

    D = lambda nu: array([[-Xuu*abs(nu[0]), 0, 0],
        [0, -Yvv*abs(nu[1]), -Yrr*abs(nu[2])],
        [0, -Nvv*abs(nu[1]), -Nrr*abs(nu[2])]], dtype=float);

    tauC = Ca(nuR).dot(nuR) + D(nuR).dot(nuR) - Ma.dot(nuCDot) - Ca(nu).dot(nu) - D(nu).dot(nu);

    return tauC
