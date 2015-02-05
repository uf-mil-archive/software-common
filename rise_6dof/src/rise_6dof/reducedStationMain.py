from __future__ import division

import threading
import traceback
import time

from numpy import *
from numpy.linalg import inv

import rospy
from rise_6dof.msg import Weights, Error, Estimate
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3Stamped
import tf
from tf import transformations
from uf_common.orientation_helpers import xyz_array

from reducedStationContinuous import reducedStationContinuousControl, reducedStationContinuousWeights


class RADPController(object):
    def __init__(self, body_frame_id, eta_c1=0.25, eta_c2=0.5, eta_a1=1, nu=0.25, beta=0.025, gamma=400):
        self.body_frame_id = body_frame_id
        
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
        auxdata.Rinv = inv(auxdata.R);

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
        
        auxdata.numPoints = 50
        
        self.weights_pub = rospy.Publisher('radp_weights', Weights, queue_size=None)
        self.error_pub = rospy.Publisher('radp_error', Error, queue_size=None)
        self.estimator_sub = rospy.Subscriber('estimate', Estimate, self.got_estimate)
        self.last_dvl_water_mass_processed = None
        self.dvl_water_mass_processed_sub = rospy.Subscriber('dvl/water_mass_processed', Vector3Stamped, self.got_dvl_water_mass_processed)
        self.tf_listener = tf.TransformListener()
        self.update_running = False
        self.stop = False
    
    def got_estimate(self, msg):
        self.theta_hat = array(msg.theta_hat).reshape((8, 1))
    
    def got_dvl_water_mass_processed(self, msg):
        try:
            trans, rot_q = self.tf_listener.lookupTransform(
                self.body_frame_id, msg.header.frame_id, rospy.Time(0))
        except:
            traceback.print_exc()
            return
        self.last_dvl_water_mass_processed = transformations.quaternion_matrix(rot_q)[:3, :3].dot(xyz_array(msg.vector))
    
    def step(self, dt, error, body_vel):
        if not hasattr(self, 'theta_hat') or self.last_dvl_water_mass_processed is None:
            return [0, 0, 0]
        
        nuC = array([body_vel[0] - self.last_dvl_water_mass_processed[0], body_vel[1] - self.last_dvl_water_mass_processed[1], 0], dtype=float).reshape((3, 1))
        
        print nuC, 'xxx'
        
        self.error = error
        self.nuC = nuC
        
        if not hasattr(self, 'etaDotC'):
            self.etaDotC = nuC # use the first nuC as etaDotC forever
        
        if not self.update_running:
            self.update_running = True
            threading.Thread(target=self.update_weights).start()
        
        u, tau_c = reducedStationContinuousControl(
            error, self.Wa1_hat, self.auxdata, self.theta_hat, nuC=nuC, etaDotC=self.etaDotC)
        
        self.error_pub.publish(Error(
            header=Header(
                stamp=rospy.Time.now(),
            ),
            zeta=map(float, error.flatten()),
            tau_c=map(float, tau_c.flatten()),
            nuC=map(float, nuC.flatten()),
        ))
        
        return u
    
    def update_weights(self):
        while not self.stop:
            time.sleep(0.2)
            dt = 0.2
            
            dWc_hat, dWa1_hat, dGamma = reducedStationContinuousWeights(
                self.error, self.Wc_hat, self.Wa1_hat, self.Gamma,
                self.auxdata, self.extrapolation_grid, self.theta_hat,
                nuC=self.nuC, etaDotC=self.etaDotC)
            
            self.Wc_hat = self.Wc_hat + dt * dWc_hat
            self.Wa1_hat = self.Wa1_hat + dt * dWa1_hat
            self.dGamma = self.Gamma + dt * dGamma
            
            self.weights_pub.publish(Weights(
                header=Header(
                    stamp=rospy.Time.now(),
                ),
                Wc_hat=self.Wc_hat,
                Wa1_hat=self.Wa1_hat,
                Gamma=self.Gamma.flatten(),
            ))
    
    def stop(self):
        self.stop = True
