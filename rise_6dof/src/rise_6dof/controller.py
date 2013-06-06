from __future__ import division

import numpy

from tf import transformations
from uf_common.orientation_helpers import quat_to_rotvec

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
    
    def update(self, dt, desired, current):
        ((desired_p, desired_o), (desired_p_dot, desired_o_dot)), ((p, o), (p_dot, o_dot)) = desired, current
        
        world_from_body = transformations.quaternion_matrix(o)[:3, :3]
        x_dot = numpy.concatenate([
            world_from_body.dot(p_dot),
            world_from_body.dot(o_dot),
        ])
        
        world_from_desiredbody = transformations.quaternion_matrix(desired_o)[:3, :3]
        desired_x_dot = numpy.concatenate([
            world_from_body.dot(desired_p_dot),
            world_from_body.dot(desired_o_dot),
        ])
        
        error_position_world = numpy.concatenate([
            desired_p - p,
            quat_to_rotvec(transformations.quaternion_multiply(
                desired_o,
                transformations.quaternion_inverse(o),
            )),
        ])
        
        error_velocity_world = (desired_x_dot + self.config['k'] * error_position_world) - x_dot
        
        if self.config['use_rise']:
            rise_term_int = self.config['ks']*self.config['alpha']*error_velocity_world + self.config['beta']*numpy.sign(error_velocity_world)
            
            self._rise_term += dt/2*(rise_term_int + self._rise_term_int_prev)
            self._rise_term_int_prev = rise_term_int
            
            output = self.config['ks'] * error_velocity_world + self._rise_term
        else:
            # zero rise term so it doesn't wind up over time
            self._rise_term = numpy.zeros(6)
            self._rise_term_int_prev = numpy.zeros(6)
            
            output = self.config['ks'] * error_velocity_world
        
        return world_from_body.T.dot(output[0:3]), world_from_body.T.dot(output[3:6])
