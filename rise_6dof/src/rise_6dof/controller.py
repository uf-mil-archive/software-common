from __future__ import division

import math

import numpy

import roslib
roslib.load_manifest('rise_6dof')
from tf import transformations
from geometry_msgs.msg import Vector3, Wrench
from uf_common.orientation_helpers import xyz_array, xyzw_array


def _jacobian(x):
    # maps body linear+angular velocities -> global linear velocity/euler rates
    sphi, cphi = math.sin(x[3]), math.cos(x[3])
    stheta, ctheta, ttheta = math.sin(x[4]), math.cos(x[4]), math.tan(x[4])
    spsi, cpsi = math.sin(x[5]), math.cos(x[5])
    
    J = numpy.zeros((6, 6))
    J[0:3, 0:3] = [
        [ ctheta * cpsi, -cphi * spsi + sphi * stheta * cpsi,  sphi * spsi + cphi * stheta * cpsi],
        [ ctheta * spsi,  cphi * cpsi + sphi * stheta * spsi, -sphi * cpsi + cphi * stheta * spsi],
        [-stheta       ,                sphi * ctheta       ,                cphi * ctheta       ],
    ]
    J[3:6, 3:6] = [
        [1, sphi * ttheta,  cphi * ttheta],
        [0, cphi         , -sphi         ],
        [0, sphi / ctheta,  cphi / ctheta],
    ]
    return J

def _jacobian_inv(x):
    # maps global linear velocity/euler rates -> body linear+angular velocities
    sphi, cphi = math.sin(x[3]), math.cos(x[3])
    stheta, ctheta = math.sin(x[4]), math.cos(x[4])
    spsi, cpsi = math.sin(x[5]), math.cos(x[5])
    
    J_inv = numpy.zeros((6, 6))
    J_inv[0:3, 0:3] = [
        [       ctheta * cpsi              ,        ctheta * spsi              ,        -stheta],
        [sphi * stheta * cpsi - cphi * spsi, sphi * stheta * spsi + cphi * cpsi,  sphi * ctheta],
        [cphi * stheta * cpsi + sphi * spsi, cphi * stheta * spsi - sphi * cpsi,  cphi * ctheta],
    ]
    J_inv[3:6, 3:6] = [
        [1,     0,       -stheta],
        [0,  cphi, sphi * ctheta],
        [0, -sphi, cphi * ctheta],
    ]
    return J_inv

class Controller(object):
    '''
    config is a dict of k, ks, alpha, beta
    
    input poses must all be in world frame/velocities must all be in body frame (follows convention from Odometry message)
    output wrench is in body frame
    '''
    
    def __init__(self, config):
        self.config = config
        
        self.reset()
    
    def reset(self):
        self._rise_term = numpy.zeros(6)
        self._rise_term_int_prev = numpy.zeros(6)
    
    def update(self, dt, desired_posetwist, current_posetwist):
        x = numpy.concatenate([xyz_array(current_posetwist.pose.position), transformations.euler_from_quaternion(xyzw_array(current_posetwist.pose.orientation))])
        x_dot = _jacobian(x).dot(numpy.concatenate([xyz_array(current_posetwist.twist.linear), xyz_array(current_posetwist.twist.angular)]))
        
        xd = numpy.concatenate([xyz_array(desired_posetwist.pose.position), transformations.euler_from_quaternion(xyzw_array(desired_posetwist.pose.orientation))])
        xd_dot = _jacobian(xd).dot(numpy.concatenate([xyz_array(desired_posetwist.twist.linear), xyz_array(desired_posetwist.twist.angular)]))
        
        
        def smallest_coterminal_angle(x):
            return (x + math.pi) % (2*math.pi) - math.pi
        error_position_world = numpy.concatenate([xd[0:3] - x[0:3], map(smallest_coterminal_angle, xd[3:6] - x[3:6])]) # e_1 in paper
        
        error_velocity_world = (xd_dot + self.config['k'] * error_position_world) - x_dot # e_2 in paper
        
        if self.config['use_rise']:
            rise_term_int = self.config['ks']*self.config['alpha']*error_velocity_world + self.config['beta']*numpy.sign(error_velocity_world)
            
            self._rise_term += dt/2*(rise_term_int + self._rise_term_int_prev)
            self._rise_term_int_prev = rise_term_int
            
            output = _jacobian_inv(x).dot(self.config['ks'] * error_velocity_world + self._rise_term)
        else:
            # zero rise term so it doesn't wind up over time
            self._rise_term = numpy.zeros(6)
            self._rise_term_int_prev = numpy.zeros(6)
            
            output = _jacobian_inv(x).dot(self.config['ks'] * error_velocity_world)
        
        
        return Wrench(force=Vector3(x=output[0], y=output[1], z=output[2]), torque=Vector3(x=output[3], y=output[4], z=output[5]))
