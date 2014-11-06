from __future__ import division

from numpy import *

def reducedStationBasis( x ):
    #Computes the value of the basis function for a given system state

    # scale = 0.2;
    # dbar = 0.05;
    # nm = [x'*x+dbar]/[1+[x'*x]];
    # d1 = scale*[1;0;1;0;1;0];
    # d2 = scale*[1;0;0;0;0;0];
    # d3 = scale*[0;1;0;0;0;0];
    # d4 = scale*[0;0;1;0;0;0];
    # d5 = scale*[0;0;0;1;0;0];
    # d6 = scale*[0;0;0;0;1;0];
    # d7 = scale*[0;0;0;0;0;1];
    # 
    # sigma = [exp[x'*[x+nm.*d1]]-1;
    #     exp[x'*[x+nm.*d2]]-1;
    #     exp[x'*[x+nm.*d3]]-1;
    #     exp[x'*[x+nm.*d4]]-1
    #     exp[x'*[x+nm.*d5]]-1
    #     exp[x'*[x+nm.*d6]]-1
    #     exp[x'*[x+nm.*d7]]-1];
    # 
    # sigma_prime = ...
    #     [[2*x+nm.*d1+2*[1-dbar]*x*d1'*x/[1+x.'*x]^2]'*exp[x'*[x+nm.*d1]];
    #     [2*x+nm.*d2+2*[1-dbar]*x*d2'*x/[1+x.'*x]^2]'*exp[x'*[x+nm.*d2]];
    #     [2*x+nm.*d3+2*[1-dbar]*x*d3'*x/[1+x.'*x]^2]'*exp[x'*[x+nm.*d3]];
    #     [2*x+nm.*d4+2*[1-dbar]*x*d4'*x/[1+x.'*x]^2]'*exp[x'*[x+nm.*d4]]
    #     [2*x+nm.*d5+2*[1-dbar]*x*d5'*x/[1+x.'*x]^2]'*exp[x'*[x+nm.*d5]]
    #     [2*x+nm.*d6+2*[1-dbar]*x*d6'*x/[1+x.'*x]^2]'*exp[x'*[x+nm.*d6]]
    #     [2*x+nm.*d7+2*[1-dbar]*x*d7'*x/[1+x.'*x]^2]'*exp[x'*[x+nm.*d7]]];

    #sigma = [x[1]*x[2] x[1]*x[3] x[1]*x[4] x[1]*x[5] x[1]*x[6] x[2]*x[3] ...
    #    x[2]*x[4] x[2]*x[5] x[2]*x[6] x[3]*x[4] x[3]*x[5] x[3]*x[6] ...
    #    x[4]*x[5] x[4]*x[6] x[5]*x[6] x[1]^2 x[2]^2 x[3]^2 x[4]^2 x[5]^2 ...
    #    x[6]^2];

    sigma_prime = zeros([21, 6])
    sigma_prime[:,0] = array([x[1], x[2], x[3], x[4], x[5], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2*x[0], 0, 0, 0, 0, 0]).transpose();
    sigma_prime[:,1] = array([x[0], 0, 0, 0, 0, x[2], x[3], x[4], x[5], 0, 0, 0, 0, 0, 0, 0, 2*x[1], 0, 0, 0, 0]).transpose();
    sigma_prime[:,2] = array([0, x[0], 0, 0, 0, x[1], 0, 0, 0, x[3], x[4], x[5], 0, 0, 0, 0, 0, 2*x[2], 0, 0, 0]).transpose();
    sigma_prime[:,3] = array([0, 0, x[0], 0, 0, 0, x[1], 0, 0, x[2], 0, 0, x[4], x[5], 0, 0, 0, 0, 2*x[3], 0, 0]).transpose();
    sigma_prime[:,4] = array([0, 0, 0, x[0], 0, 0, 0, x[1], 0, 0, x[2], 0, x[3], 0, x[5], 0, 0, 0, 0, 2*x[4], 0]).transpose();
    sigma_prime[:,5] = array([0, 0, 0, 0, x[0], 0, 0, 0, x[1], 0, 0, x[2], 0, x[3], x[4], 0, 0, 0, 0, 0, 2*x[5]]).transpose();
    
    return sigma_prime
