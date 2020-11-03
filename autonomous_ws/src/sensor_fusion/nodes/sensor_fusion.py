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
        #First three is GPS, next 3 is IMU acc, next 3 is IMU gyro
        self.data_idx = np.zeros(9, dtype=np.int8)
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.linear_acc = 0
        self.angular_vel = 0
        self.xhat = np.zeros(3)
        
    def setup_subs(self):
        self.pos_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.pos_cb)
        self.imu_sub = rospy.Subscriber('mavros/imu/data_raw', Imu, self.imu_cb)
    
    def pos_cb(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.alt = msg.altitude
        self.data_idx[:3] = 1

    def imu_cb(self, msg):
        self.linear_acc = msg.linear_acceleration
        self.angular_vel = msg.angular_velocity
        self.data_idx[3:9] = 1

    def filter(self):
        #Run Kalman filter
        #prediction step
        self.xhat = self.xhat + 
        #update step


    

if __name__ == "__main__":
    sf = SensorFusion()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rospy.spin_once()
        sf.filter()
        rate.sleep()