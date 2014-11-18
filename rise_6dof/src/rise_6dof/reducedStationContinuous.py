from __future__ import division

from numpy import *
from numpy.linalg import inv

from model2d import model2d
from reducedStationBasis import reducedStationBasis
from estimator import estimator as reducedStationEstimator

def reducedStationContinuous(zeta, Wc_hat, Wa1_hat, gamma, auxdata, extrapolation_grid, theta_hat):
    #[f, g] = model2d(zeta);
    [Y,f0,g] = reducedStationEstimator(zeta);

    sp = reducedStationBasis(zeta);

    u1 = -0.5*(inv(auxdata.R).dot(g.transpose())).dot(sp.transpose()).dot(Wa1_hat);

    #Fu = f + g.dot(u1);
    Fu = Y.dot(theta_hat) + f0 + g.dot(u1);

    omega = sp.dot(Fu);

    delta = zeta.transpose().dot(auxdata.Q).dot(zeta) + u1.transpose().dot(auxdata.R).dot(u1) + Wc_hat.transpose().dot(omega);

    # p = sqrt(1 + omega.transpose().dot(omega));
    p = 1 + (auxdata.nu * omega.transpose()).dot(gamma).dot(omega);

    #-------------------------------------------------------------------------#
    #------------------ Build Bellman Extrapolation Library ------------------#
    #-------------------------------------------------------------------------#

    # samples = sample_grid + repmat(zeta,1,auxdata.numPoints);
    extrapolation_sum = zeros([21, 1]);

    # extrapolation_grid = scale*randn(6,auxdata.numPoints);

    for i in xrange(auxdata.numPoints):
        eg_col = array([extrapolation_grid[:,i]]).transpose()
        #f_i, g_i = model2d(eg_col);
        
        # Bellman Error Stack
        [Y_i,f0_i,g_i] = reducedStationEstimator(extrapolation_grid[:,i]);
        
        sp_i = reducedStationBasis(eg_col);
        
        u1_i = -0.5*(inv(auxdata.R).dot(g_i.transpose())).dot(sp_i.transpose()).dot(Wa1_hat);
        
        Wu_i = u1_i.transpose().dot(auxdata.R).dot(u1_i);
        
        #Fu_i = f_i + g_i.dot(u1_i);
        Fu_i = Y_i.dot(theta_hat) + f0_i + g_i.dot(u1_i);
        
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

    dWa1_hat = -auxdata.eta_a1*(Wa1_hat - Wc_hat)*(auxdata.WaLimit > Wa1_hat);

    return [u1, dWc_hat, dWa1_hat, dGamma]
