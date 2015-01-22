from __future__ import division

from numpy import *

from model2d import model2d
from reducedStationBasis import reducedStationBasis
from estimator import estimator as reducedStationEstimator
from bodyCurrent import bodyCurrent
from currentCompensator import currentCompensator

def reducedStationContinuousControl(zeta, Wa_hat, auxdata, theta_hat, nuC, etaDotC):
    nuCDot = cross(nuC.reshape(3), [0, 0, zeta[5]]).reshape((3, 1)) # XXX take numerical derivative
    etaDDotC = cross(etaDotC.reshape(3), [0, 0, zeta[5]]).reshape((3, 1))
    
    if auxdata.constantCurrent:
        [nuC, nuCDot] = bodyCurrent(zeta[2], zeta[5], etaDotC, etaDDotC);
    
    #[f, g] = model2d(zeta);
    [Y1,Y2,Y3,Y4,f0,f1,g] = reducedStationEstimator(zeta, nuC, nuCDot);

    sp = reducedStationBasis(zeta);

    u = -0.5*(auxdata.Rinv.dot(g.transpose())).dot(sp.transpose()).dot(Wa_hat);

    if auxdata.constantCurrent:
        u_ss = Y3.dot(theta_hat);
        tau_b = u_ss + u;
        
        Fu = (Y1+Y2).dot(theta_hat) + f0 + g.dot(u);
    else:
        tau_c = currentCompensator(theta_hat, zeta[3:6], nuC, nuCDot);
        tau_b = u + tau_c;
        
        Fu = (Y4).dot(theta_hat) + f0 + g.dot(u);
    
    return tau_b

def reducedStationContinuousWeights(zeta, Wc_hat, Wa_hat, gamma, auxdata, extrapolation_grid, theta_hat, nuC, etaDotC):
    nuCDot = cross(nuC.reshape(3), [0, 0, zeta[5]]).reshape((3, 1)) # XXX take numerical derivative
    etaDDotC = cross(etaDotC.reshape(3), [0, 0, zeta[5]]).reshape((3, 1))
    
    if auxdata.constantCurrent:
        [nuC, nuCDot] = bodyCurrent(zeta[2], zeta[5], etaDotC, etaDDotC);
    
    #[f, g] = model2d(zeta);
    [Y1,Y2,Y3,Y4,f0,f1,g] = reducedStationEstimator(zeta, nuC, nuCDot);

    sp = reducedStationBasis(zeta);

    u = -0.5*(auxdata.Rinv.dot(g.transpose())).dot(sp.transpose()).dot(Wa_hat);

    if auxdata.constantCurrent:
        u_ss = Y3.dot(theta_hat);
        tau_b = u_ss + u;
        
        Fu = (Y1+Y2).dot(theta_hat) + f0 + g.dot(u);
    else:
        tau_c = currentCompensator(theta_hat, zeta[3:6], nuC, nuCDot);
        tau_b = u + tau_c;
        
        Fu = (Y4).dot(theta_hat) + f0 + g.dot(u);

    omega = sp.dot(Fu);

    delta = zeta.transpose().dot(auxdata.Q).dot(zeta) + u.transpose().dot(auxdata.R).dot(u) + Wc_hat.transpose().dot(omega);

    # p = sqrt(1 + omega.transpose().dot(omega));
    p = 1 + (auxdata.nu * omega.transpose()).dot(gamma).dot(omega);

    #-------------------------------------------------------------------------#
    #------------------ Build Bellman Extrapolation Library ------------------#
    #-------------------------------------------------------------------------#

    # samples = sample_grid + repmat(zeta,1,auxdata.numPoints);
    extrapolation_sum = zeros([21, 1]);

    # extrapolation_grid = scale*randn(6,auxdata.numPoints);

    for i in xrange(auxdata.numPoints):
        # get a column vector. extrapolation_grid[:,i] yields a 1D array
        eg_col = array([extrapolation_grid[:,i]]).transpose()
        #f_i, g_i = model2d(eg_col);
        
        # Bellman Error Stack
        if auxdata.constantCurrent:
            [nuC_extrap, nuCDot_extrap] = bodyCurrent(
                extrapolation_grid[2,i], extrapolation_grid[5,i],
                etaDotC, etaDDotC);
        else:
            nuC_extrap = nuCDot_extrap = zeros((3, 1))
        
        [Y1_i,Y2_i,_,Y4_i,f0_i,_,g_i] = \
            reducedStationEstimator(eg_col, nuC_extrap,
            nuCDot_extrap);
        
        sp_i = reducedStationBasis(eg_col);
        
        u_i = -0.5*(auxdata.Rinv.dot(g_i.transpose())).dot(sp_i.transpose()).dot(Wa_hat);
        
        Wu_i = u_i.transpose().dot(auxdata.R).dot(u_i);
        
        if auxdata.constantCurrent:
            Fu_i = (Y1_i+Y2_i).dot(theta_hat) + f0_i + g_i.dot(u_i);
        else:
            Fu_i = (Y4_i).dot(theta_hat) + f0_i + g_i.dot(u_i);
        
        omega_i = sp_i.dot(Fu_i);
        
        delta_i = eg_col.transpose().dot(auxdata.Q).dot(eg_col) + \
            Wu_i + Wc_hat.transpose().dot(omega_i);
        
        # p_i = sqrt(1 + omega_i.transpose().dot(omega_i));
        p_i = 1 + (auxdata.nu * omega_i.transpose()).dot(gamma).dot(omega_i);

        extrapolation_sum = extrapolation_sum + (delta_i/p_i)*(omega_i);

    # dWc_hat = (-auxdata.eta_c1.dot(delta)/p).dot(omega) -  ...
    #     (auxdata.eta_c2/auxdata.nodes).dot(extrapolation_sum);
    dWc_hat = (-auxdata.eta_c1.dot(gamma)*delta/p).dot(omega) -  \
        (auxdata.eta_c2/auxdata.numPoints).dot(gamma).dot(extrapolation_sum);
    dGamma = (auxdata.beta*(gamma) - \
        auxdata.eta_c1.dot(gamma).dot(omega.dot(omega.transpose())/p**2).dot(gamma)).dot(auxdata.gammaLimit > gamma);

    dWa_hat = -auxdata.eta_a1*(Wa_hat - Wc_hat)*(auxdata.WaLimit > Wa_hat);

    return [dWc_hat, dWa_hat, dGamma]
