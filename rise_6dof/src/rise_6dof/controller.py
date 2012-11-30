from __future__ import division

import math

import numpy

import roslib
roslib.load_manifest('rise_6dof')
from tf import transformations
from geometry_msgs.msg import Vector3, Wrench


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
    
    inputs must all be in world frame
    output wrench is in body frame
    '''
    
    def __init__(self, config):
        self.config = config
        
        self._rise_term = numpy.zeros(6)
        self._rise_term_int_prev = numpy.zeros(6)
    
    def update(self, dt, desired_posetwist, current_posetwist):
        xyz_array = lambda p: numpy.array([p.x, p.y, p.z])
        xyzw_array = lambda q: numpy.array([q.x, q.y, q.z, q.w])
        
        body_from_world = transformations.quaternion_matrix(transformations.quaternion_conjugate(xyzw_array(current_posetwist.pose.orientation)))[:3, :3]
        
        x = numpy.concatenate([xyz_array(current_posetwist.pose.position), transformations.euler_from_quaternion(xyzw_array(current_posetwist.pose.orientation))])
        vb = numpy.concatenate([body_from_world.dot(xyz_array(current_posetwist.twist.linear)), body_from_world.dot(xyz_array(current_posetwist.twist.angular))])
        
        xd = numpy.concatenate([xyz_array(desired_posetwist.pose.position), transformations.euler_from_quaternion(xyzw_array(desired_posetwist.pose.orientation))])
        xd_dot = numpy.concatenate([xyz_array(desired_posetwist.twist.linear), _jacobian(x)[3:6,3:6].dot(body_from_world.dot(xyz_array(desired_posetwist.twist.angular)))])
        
        
        def smallest_coterminal_angle(x):
            return (x + math.pi) % (2*math.pi) - math.pi
        error_position_world = numpy.concatenate([xd[0:3] - x[0:3], map(smallest_coterminal_angle, xd[3:6] - x[3:6])]) # e in paper
        
        desired_velocity_world = self.config['k'] * error_position_world + xd_dot
        desired_velocity_body = _jacobian_inv(x).dot(desired_velocity_world)
        
        error_velocity_body = desired_velocity_body - vb # e2 in paper
        
        
        if self.config['use_rise']:
            ksPlus1 = self.config['ks'] + numpy.ones(6)
            
            sign = lambda x: 1 if x > 0 else -1 if x < 0 else 0
            rise_term_int = ksPlus1*self.config['alpha']*error_velocity_body + self.config['beta']*map(sign, error_velocity_body)
            
            self._rise_term += dt/2*(rise_term_int + self._rise_term_int_prev)
            self._rise_term_int_prev = rise_term_int
            
            output = ksPlus1 * error_velocity_body + self._rise_term
        else:
            # zero rise term so it doesn't wind up over time
            self._rise_term = numpy.zeros(6)
            self._rise_term_int_prev = numpy.zeros(6)
            
            output = self.config['ks'] * error_velocity_body
        
        
        return Wrench(force=Vector3(x=output[0], y=output[1], z=output[2]), torque=Vector3(x=output[3], y=output[4], z=output[5]))
