#!/usr/bin/env python3

import rospy
import mavros
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Vector3

import numpy as np
from math import pi, sqrt, atan2
import matplotlib.pyplot as plt


class SensorFusion:
    def __init__(self):
        self.setup_subs()
        self.dt = 0.02
        #First three is GPS, next 3 is IMU acc, next 3 is IMU gyro
        self.data_idx = np.zeros(9, dtype=np.int8)
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.u = np.zeros((3,1))
        self.y = np.zeros((3,1))
        self.xhat = np.zeros((3,1))
        self.sigma = np.identity(self.xhat.shape[0])*10000
        self.Q = np.identity(self.u.shape[0])
        self.R = np.identity(self.y.shape[0])*0.02
        self.C = np.identity(self.y.shape[0])

        self.setup_state_model()
        
    def setup_subs(self):
        self.pos_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.pos_cb)
        self.imu_sub = rospy.Subscriber('mavros/imu/data_raw', Imu, self.imu_cb)
    
    def pos_cb(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.alt = msg.altitude
        self.data_idx[:3] = 1
        self.y[:3] = np.asarray([msg.latitude, msg.longitude, msg.altitude]).reshape(3,1)

    def imu_cb(self, msg):
        self.u[:3] = np.asarray([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]).reshape(3,1)
        #self.u[3:] = np.asarray([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
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