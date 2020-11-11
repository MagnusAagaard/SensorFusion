#!/usr/bin/env python3
import sys
import os
import rospy
import mavros
from sensor_msgs.msg import NavSatFix, Imu

import numpy as np
from math import pi, sqrt, atan2, cos, sin
import matplotlib.pyplot as plt

from utm import utmconv


class SensorFusion:
    def __init__(self):
        self.setup_subs()
        self.uc = utmconv()
        self.dt = 0.02
        #First three is GPS, next 3 is IMU acc, next 3 is IMU gyro
        self.data_idx = np.zeros(9, dtype=np.int8)
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.u = np.zeros((6,1))
        self.y = np.zeros((3,1))
        self.xhat = np.zeros((3,1))
        self.sigma = np.identity(self.xhat.shape[0])*10000
        self.Q = np.identity(self.u.shape[0])
        self.R = np.identity(self.y.shape[0])*0.02
        self.C = np.identity(self.y.shape[0])
        self.setup_state_model()
        ## Nyt stuff:
        self.xh = self.init_navigation_state()
        self.delta_u_h = np.zeros((6,1))
        (self.P, self.Q1, self.Q2, _, _) = self.init_filter()
        
        
        
    def setup_subs(self):
        self.pos_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.pos_cb)
        self.imu_sub = rospy.Subscriber('mavros/imu/data_raw', Imu, self.imu_cb)
    
    def pos_cb(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.alt = msg.altitude
        (hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm(self.lat,self.lon)
        self.data_idx[:3] = 1
        self.y[:3] = np.asarray([e1, n1, msg.altitude]).reshape(3,1)

    def imu_cb(self, msg):
        self.u = np.asarray([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]).reshape(6,1)
        self.data_idx[3:9] = 1

    def setup_state_model(self):
        self.A = np.identity(self.xhat.shape[0])
        #self.A[0][3] = self.dt
        #self.A[1][4] = self.dt
        #self.A[2][5] = self.dt
        B1 = np.identity(3)*0.5*self.dt*self.dt
        self.B = B1
        #B2 = np.identity(3)*self.dt
        #self.B = np.concatenate((B1,B2))

    def init_navigation_state(self):
        roll = 0
        pitch = 0
        heading = 0
        # Init coord. rotational matrix
        Rb2t = self.Rt2b([roll, pitch, heading])
        q = dcm2q(Rb2t)
        x_h = np.concatenate((np.zeros((6,1)), q))
        return x_h

    def Rt2b(self, ang):
        #function for calculation of the rotation matrix for rotaion from tangent frame to body frame.
        cr = cos(ang[0])
        sr = sin(ang[0])
        cp = cos(ang[1])
        sp = sin(ang[1])
        cy = cos(ang[2])
        sy = sin(ang[2])
        R = np.asarray([[cy*cp, sy*cp -1*sp],[-1*sy*cr+cy*sp*sr, cy*cr+sy*sp*sr, cp*sr], [sy*sr+cy*sp*cr, -1*cy*sr+sy*sp*cr, cp*cr]])
        return R

    def dcm2q(self, R):
        # Function for transformation from directional cosine matrix to quaternions
        q = np.zeros((4,1))
        np.diag()
        q[3] = 0.5*sqrt(1+np.sum(np.diag(R)))
        q[0] = (R[2,1]-R[1,2])/(4*q[3])
        q[1] = (R[0,2]-R[2,0])/(4*q[3])
        q[2] = (R[1,0]-R[0,1])/(4*q[3])
        return q

    def init_filter(self):
        P = np.zeros((15,15))
        # Initial Kalman filter uncertainties (standard deviations)
        P[0:3,0:3] = pow(10,2)*np.eye(3)    # Position = 10 [m]
        P[3:6,3:6] = pow(5,2)*np.eye(3)     # Velocity = 5 [m/s]
        factp = np.asarray([pow(1*pi/180,2), pow(1*pi/180,2), pow(20*pi/180,2)])
        P[6:9,6:9] = np.diag(factp)    # Attitude (roll, pitch, yaw) [rad]
        P[9:12,9:12] = pow(0.02,2)*np.eye(3)     # Accelerometer biases [m/s^2]
        P[12:,12:] = pow((0.05*pi/180),2)*np.eye(3) # Gyro biases [rad/s]
        # Process noise covariance
        Q1 = np.zeros((6,6))
        Q1[0:3,0:3] = np.diag(pow(0.05,2)*np.eye(3))        # sigma acc
        Q1[3:,3:] = np.diag(pow(0.1*pi/180,2)*np.eye(3))    # sigma gyro
        Q2 = np.zeros((6,6))
        Q2[0:3,0:3] = np.eye(3)* pow(0.0001,2)              # sigma acc bias
        Q2[3:,3:] = np.eye(3)*pow(0.01*pi/180,2)            # sigma gyro bias
        R = np.eye(3)*pow(3/sqrt(3),2)                      # GNSS-receiver position measurement noise
        # Observation matrix
        H = np.concatenate((np.eye(3), np.zeros((3,12))), axis=1)
        return (P,Q1,Q2,R,H)


    def filter(self):
        print("filter")
        #Run Kalman filter

        #prediction step
        self.xhat = self.A.dot(self.xhat)+self.B.dot(self.u)
        self.sigma = np.matmul(np.matmul(self.A, self.sigma),np.transpose(self.A)) + self.Q
        #update step
        tmp = np.linalg.inv(np.matmul(np.matmul(self.C, self.sigma), np.transpose(self.C)) + self.R)
        K = np.matmul(np.matmul(self.sigma, np.transpose(self.C)),tmp)
        self.xhat = self.xhat + np.matmul(K,(self.y - np.matmul(self.C, self.xhat)))
        self.sigma = np.matmul((np.identity(self.xhat.shape[0]) - np.matmul(K,self.C)),self.sigma)


    

if __name__ == "__main__":
    rospy.init_node('sensor_fusion_node', anonymous=True)
    sf = SensorFusion()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        sf.filter()
        rate.sleep()
    rospy.spin()