from __future__ import division

from numpy import *

from jacobi2d import jacobi2d

def bodyCurrent(psi, psiDot, etaDotC, etaDDotC):
    u = etaDotC[0];
    v = etaDotC[1];
    uDot = etaDDotC[0];
    vDot = etaDDotC[1];

    nuC = jacobi2d(psi).transpose().dot(etaDotC);
    nuCDot = array([
        [cos(psi)*uDot-sin(psi)*psiDot*u+cos(psi)*psiDot*v+sin(psi)*vDot],
        [cos(psi)*vDot-cos(psi)*psiDot*u-sin(psi)*uDot-sin(psi)*psiDot*v],
        [0],
    ], dtype=float);

    return nuC, nuCDot
