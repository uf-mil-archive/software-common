from __future__ import division

import math

import numpy
numpy.set_printoptions(linewidth=100000000000)

import roslib
roslib.load_manifest('kalman_6dof')
import rospy
from tf import transformations
from uf_common.orientation_helpers import rotvec_to_quat, quat_to_rotvec


G = numpy.array([0, 0, -9.81]) # gravity acceleration vector

skewmatrix = lambda (x, y, z): numpy.array([[0,-z,y],
                                            [z,0,-x],
                                            [-y,x,0]])


class Kalman(object):
    def __init__(self, t, # t is a rospy.Time
            position, orientation, velocity_body, P,
            angular_velocity_body, angular_velocity_body_cov,
            acceleration_felt_body, acceleration_felt_body_cov):
        assert .9 <= numpy.linalg.norm(orientation) <= 1.1
        self.t = t
        self.position = position
        self.orientation = orientation/numpy.linalg.norm(orientation)
        self.velocity_body = velocity_body
        self.P = P
        self.angular_velocity_body = angular_velocity_body
        self.angular_velocity_body_cov = angular_velocity_body_cov
        self.acceleration_felt_body = acceleration_felt_body
        self.acceleration_felt_body_cov = acceleration_felt_body_cov
    
    def _predict(self, t,
            angular_velocity_body, angular_velocity_body_cov,
            acceleration_felt_body, acceleration_felt_body_cov,
            measurement_t, _return_debug_matrices=False):
        
        angular_velocity_body = numpy.array(angular_velocity_body)
        acceleration_felt_body = numpy.array(acceleration_felt_body)
        
        dt = (t - self.t).to_sec()
        assert dt >= 0, dt
        # XXX skip if dt == 0
        
        dt_measurement = (measurement_t - self.t).to_sec()
        
        # this looks a little over-complicated because I tried to keep the frame that the accelerometer is in correct while doing midpoint integration
        
        oldbody_from_newbody_q = rotvec_to_quat(dt * angular_velocity_body)
        oldbody_from_newbody = transformations.quaternion_matrix(oldbody_from_newbody_q)[:3, :3]
        orientation = transformations.quaternion_multiply(self.orientation, oldbody_from_newbody_q)
        
        oldbody_from_accelbody_q = rotvec_to_quat(dt_measurement * angular_velocity_body)
        oldbody_from_accelbody = transformations.quaternion_matrix(oldbody_from_accelbody_q)[:3, :3]
        
        world_from_oldbody = transformations.quaternion_matrix(self.orientation)[:3, :3]
        world_from_newbody = transformations.quaternion_matrix(orientation)[:3, :3]
        world_from_accelbody = world_from_oldbody.dot(oldbody_from_accelbody)
        
        acceleration_felt_accelbody = acceleration_felt_body
        acceleration_felt = world_from_accelbody.dot(acceleration_felt_accelbody)
        acceleration = acceleration_felt + G
        position = self.position + dt * world_from_oldbody.dot(self.velocity_body) + dt**2/2 * acceleration
        
        velocity_oldbody = self.velocity_body + dt * (oldbody_from_accelbody.dot(acceleration_felt_accelbody) + world_from_oldbody.T.dot(G))
        velocity_newbody = oldbody_from_newbody.T.dot(velocity_oldbody)
        
        F = numpy.zeros((9, 9))
        F[0:3, 0:3] = numpy.identity(3)
        F[0:3, 3:6] = dt * skewmatrix(world_from_oldbody.dot(self.velocity_body)).T + dt**2/2 * skewmatrix(acceleration_felt).T # orientation affecting position
        F[0:3, 6:9] = dt * world_from_oldbody # velocity affecting position
        
        F[3:6, 3:6] = numpy.identity(3) # orientation error carries over since it's in world coordinates
        
        F[6:9, 3:6] = dt * world_from_newbody.T.dot(skewmatrix(G)) # orientation affecting velocity
        F[6:9, 6:9] = oldbody_from_newbody.T # velocity affecting velocity
        
        L = numpy.zeros((9, 6))
        J_acceleration_newbody_angular_velocity_body = skewmatrix(acceleration_felt_body).T * dt_measurement
        L[0:3, 0:3] += dt**2/2 * world_from_newbody.dot(J_acceleration_newbody_angular_velocity_body) # gyro noise affecting position
        L[6:9, 0:3] += dt * J_acceleration_newbody_angular_velocity_body # gyro noise affecting velocity
        
        L[0:3, 3:6] += dt**2/2 * world_from_accelbody # accel noise affecting position
        L[3:6, 0:3] += dt * world_from_oldbody # gyro noise affecting orientation XXX approximation
        L[6:9, 0:3] += dt * oldbody_from_newbody.T.dot(skewmatrix(velocity_oldbody)) # gyro noise affecting velocity XXX approximation
        L[6:9, 3:6] += dt * oldbody_from_newbody.T.dot(oldbody_from_accelbody) # accel noise affecting velocity
        
        Q = numpy.zeros((6, 6))
        Q[0:3, 0:3] += angular_velocity_body_cov
        Q[3:6, 3:6] += acceleration_felt_body_cov
        
        P = F.dot(self.P).dot(F.T) + L.dot(Q).dot(L.T)
        
        k = Kalman(t,
            position, orientation, velocity_newbody, P,
            angular_velocity_body, angular_velocity_body_cov,
            acceleration_felt_body, acceleration_felt_body_cov)
        
        if _return_debug_matrices:
            return k, F, L
        else:
            return k
    
    def predict(self, t,
            angular_velocity_body, angular_velocity_body_cov,
            acceleration_felt_body, acceleration_felt_body_cov):
        assert t >= self.t
        mid_t = self.t + (t - self.t)/2
        
        # midpoint integration
        # use previous data up to time centered between IMU measurements and then new data from center time to new time
        return self._predict(mid_t, self.angular_velocity_body, self.angular_velocity_body_cov, self.acceleration_felt_body, self.acceleration_felt_body_cov, self.t) \
                   ._predict(    t,      angular_velocity_body,      angular_velocity_body_cov,      acceleration_felt_body,      acceleration_felt_body_cov,      t)
    
    def update(self, measurement_minus_predicted, prediction_jacobian, measurement_covariance, debug=False, name=None):
        S = prediction_jacobian.dot(self.P).dot(prediction_jacobian.T) + measurement_covariance
        K = self.P.dot(prediction_jacobian.T).dot(numpy.linalg.inv(S))
        if debug: print 'P', self.P
        if debug: print 'H', prediction_jacobian
        if debug: print 'S-', numpy.linalg.inv(S)
        if debug: print 'K', K
        dx = K.dot(measurement_minus_predicted).flatten()
        if debug: print 'dx', dx
        if debug: print 'Q', (numpy.identity(9) - K.dot(prediction_jacobian)).dot(self.P)
        if name is not None:
            print name, ' '.join('%8.5f' % _ for _ in dx)
        return self._apply_dx(dx, (numpy.identity(9) - K.dot(prediction_jacobian)).dot(self.P))
    
    def _apply_dx(self, dx, P=None):
        if P is None:
            P = self.P
        return Kalman(
            t=self.t,
            position=self.position + dx[0:3],
            orientation=transformations.quaternion_multiply(rotvec_to_quat(dx[3:6]), self.orientation),
            velocity_body=self.velocity_body + dx[6:9],
            P=P,
            angular_velocity_body=self.angular_velocity_body,
            angular_velocity_body_cov=self.angular_velocity_body_cov,
            acceleration_felt_body=self.acceleration_felt_body,
            acceleration_felt_body_cov=self.acceleration_felt_body_cov,
        )
    
    def _get_dx(self, other):
        q = transformations.quaternion_multiply(self.orientation, transformations.quaternion_conjugate(other.orientation))
        return self.position - other.position, quat_to_rotvec(q), self.velocity_body - other.velocity_body
