from __future__ import division

from numpy import *

import rospy
from rise_6dof.msg import Weights, Error, Estimate
from std_msgs.msg import Header

from reducedStationContinuous import reducedStationContinuous


class RADPController(object):
    def __init__(self, eta_c1=0.25, eta_c2=0.5, eta_a1=1, nu=0.25, beta=0.025, gamma=400):
        class Object(object): pass
        auxdata = Object()
        
        auxdata.constantCurrent = False

        auxdata.nodes = 21;
        auxdata.gridSize = 3;
        auxdata.numPoints = auxdata.gridSize**4*(auxdata.gridSize+2)**2;

        auxdata.WaLimit = 1e4*ones([auxdata.nodes,1]);

        # Extrapolation Gradient Laws
        auxdata.eta_c1 = eta_c1*eye(auxdata.nodes);
        auxdata.eta_c2 = eta_c2*eye(auxdata.nodes);
        auxdata.eta_a1 = eta_a1;

        # Extrapolation Least Squares
        auxdata.nu = nu;
        auxdata.beta  = beta;
        auxdata.gammaLimit = 1e4*eye(auxdata.nodes);
        self.Gamma = gamma*eye(auxdata.nodes);

        # Optimal Weights
        auxdata.Q = 10*diag([2, 5, 2, 1, 1, 1]);
        auxdata.R = eye(3);

        # Linear System Weights
        W_hat0 = zeros([auxdata.nodes,1]);
        # W_hat0(3) = 200.8;
        # W_hat0(6) = -2.69;
        # W_hat0(8) = 419.4;
        # W_hat0(9) = -12.21;
        # W_hat0(11) = -12.21;
        # W_hat0(12) = 52.6;
        # W_hat0(15) = -87.0;
        # W_hat0(16) = 45.91;
        # W_hat0(17) = 65.52;
        # W_hat0(18) = 25.0;
        # W_hat0(19) = 461.0;
        # W_hat0(20) = 1379;
        # W_hat0(21) = 66.68;

        W_hat0[15] = 93.61;
        W_hat0[2] = 428.11;
        W_hat0[16] = 193.87;
        W_hat0[5] = -0.6663;
        W_hat0[7] = 741.68;
        W_hat0[8] = -2.16;
        W_hat0[17] = 130.68;
        W_hat0[10] =  -2.16;
        W_hat0[11] = 331.54;
        W_hat0[18] = 1001.84;
        W_hat0[19] = 1437.90;
        W_hat0[14] = -10.59;
        W_hat0[20] = 433.26;
        
        extrapolation_grid = zeros([6,auxdata.numPoints]);
        count = 0;
        for i in linspace(-0.2,0.2,auxdata.gridSize+2):
            for j in linspace(-0.2,0.2,auxdata.gridSize+2):
                for k in linspace(-0.05,0.05,auxdata.gridSize):
                    for l in linspace(-0.1,0.1,auxdata.gridSize):
                        for m in linspace(-0.1,0.1,auxdata.gridSize):
                            for n in linspace(-0.05,0.05,auxdata.gridSize):
                                extrapolation_grid[:,count] = array([i, j, k, l, m, n]).transpose();
                                count = count + 1;
        
        self.Wc_hat = array(W_hat0)
        self.Wa1_hat = array(W_hat0)
        self.auxdata = auxdata
        self.extrapolation_grid = extrapolation_grid
        
        auxdata.numPoints = 106
        
        self.weights_pub = rospy.Publisher('radp_weights', Weights)
        self.error_pub = rospy.Publisher('radp_error', Error)
        self.estimator_sub = rospy.Subscriber('estimate', Estimate, self.got_estimate)
    
    def got_estimate(self, msg):
        self.theta_hat = array(msg.theta_hat).reshape((8, 1))
    
    def step(self, dt, error):
        if not hasattr(self, 'theta_hat'):
            return [0, 0, 0]
        
        self.error_pub.publish(Error(
            header=Header(
                stamp=rospy.Time.now(),
            ),
            zeta=map(float, error.flatten()),
        ))
        
        self.weights_pub.publish(Weights(
            header=Header(
                stamp=rospy.Time.now(),
            ),
            Wc_hat=self.Wc_hat,
            Wa1_hat=self.Wa1_hat,
            Gamma=self.Gamma.flatten(),
        ))
        
        u1, dWc_hat, dWa1_hat, dGamma = reducedStationContinuous(
            error, self.Wc_hat, self.Wa1_hat, self.Gamma,
            self.auxdata, self.extrapolation_grid, self.theta_hat,
            nuC, nuCDot, etaDotC, etaDDotC)
        
        self.Wc_hat = self.Wc_hat + dt * dWc_hat
        self.Wa1_hat = self.Wa1_hat + dt * dWa1_hat
        self.dGamma = self.Gamma + dt * dGamma
        
        return u1
