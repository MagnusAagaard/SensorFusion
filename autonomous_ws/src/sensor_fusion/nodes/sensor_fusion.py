#!/usr/bin/env python3
import sys
import os
import rospy
import mavros
from sensor_msgs.msg import NavSatFix, Imu

import numpy as np
from scipy.linalg import block_diag
from math import pi, sqrt, atan2, cos, sin
import matplotlib.pyplot as plt

from utm import utmconv


class SensorFusion:
    def __init__(self):
        self.uc = utmconv()
        #settings
        self.sigma_gps = 3/sqrt(3)
        self.sigma_non_holonomic = 20

        self.dt = 0.02
        self.y = np.zeros((3,1))
        self.data_idx = np.zeros(5, dtype=np.int8)
        self.data_idx[3:5] = 1
        msg = rospy.wait_for_message('mavros/imu/data_raw', Imu)
        self.u = np.asarray([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]).reshape(6,1)
        #self.u = np.zeros((6,1))
        self.xh = self.init_navigation_state()
        self.delta_u_h = np.zeros((6,1))
        (self.P, self.Q1, self.Q2, _, _) = self.init_filter()
        self.setup_subs()
        self.xh_s = []
        self.gps_gt = []
        self.imu_data = []
        
        
    def setup_subs(self):
        self.pos_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.pos_cb)
        self.imu_sub = rospy.Subscriber('mavros/imu/data_raw', Imu, self.imu_cb)
    
    def pos_cb(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.alt = msg.altitude
        (hemisphere, zone, letter, e1, n1) = self.uc.geodetic_to_utm(self.lat,self.lon)
        self.data_idx[:3] = 1
        self.y[:3] = np.asarray([e1, n1, msg.altitude]).reshape(3,1)

    def imu_cb(self, msg):
        self.u = np.asarray([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]).reshape(6,1)
        self.imu_data.append(self.u.flatten())

    def init_navigation_state(self):
        roll = 0
        pitch = 0
        heading = 0
        # Init coord. rotational matrix
        Rb2t = np.array(self.Rt2b([roll, pitch, heading]))
        q = self.dcm2q(Rb2t)
        # Få inital pos estimate her og smid ind!
        msg = rospy.wait_for_message('/mavros/global_position/global', NavSatFix)
        self.lat = msg.latitude
        self.lon = msg.longitude
        (hemisphere, zone, letter, e1, n1) = self.uc.geodetic_to_utm(self.lat,self.lon)
        initial_pos = np.array([e1,n1, msg.altitude]).reshape(3,1)
        x_h = np.concatenate((initial_pos,np.zeros((3,1)), q))
        return x_h

    def Rt2b(self, ang):
        #function for calculation of the rotation matrix for rotaion from tangent frame to body frame.
        cr = cos(ang[0])
        sr = sin(ang[0])
        cp = cos(ang[1])
        sp = sin(ang[1])
        cy = cos(ang[2])
        sy = sin(ang[2])
        R = np.asarray([[cy*cp, sy*cp, -1*sp],[-1*sy*cr+cy*sp*sr, cy*cr+sy*sp*sr, cp*sr], [sy*sr+cy*sp*cr, -1*cy*sr+sy*sp*cr, cp*cr]])
        return R

    def dcm2q(self, R):
        # Function for transformation from directional cosine matrix to quaternions
        q = np.zeros((4,1))
        q[3] = 0.5*sqrt(1+np.sum(np.diag(R)))
        q[0] = (R[2,1]-R[1,2])/(4*q[3])
        q[1] = (R[0,2]-R[2,0])/(4*q[3])
        q[2] = (R[1,0]-R[0,1])/(4*q[3])
        return q

    def init_filter(self):
        P = np.zeros((15,15))
        # Initial Kalman filter uncertainties (standard deviations)
        P[0:3,0:3] = pow(1,2)*np.eye(3)    # Position = 10 [m]
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

    def q2dcm(self, q):
        # Function for transformation from quaternions to directional cosine matrix
        p = np.zeros((6,1))
        p[0:4] = pow(q[0:4],2)
        p[4] = p[1] + p[2]
        if p[0] + p[3] + p[4] != 0:
            p[5] = 2/(p[0]+p[3]+p[4])
        else:
            p[5] = 0
        R = np.zeros((3,3))
        R[0,0] = 1-p[5]*p[4]
        R[1,1] = 1-p[5]*(p[0]+p[2])
        R[2,2] = 1-p[5]*(p[0]+p[1])
        
        p[0] = p[5]*q[0]
        p[1] = p[5]*q[1]
        p[4] = p[5]*q[2]*q[3]
        p[5] = p[0]*q[1]

        R[0,1] = p[5] - p[4]
        R[1,0] = p[5] + p[4]

        p[4] = p[1]*q[3]
        p[5] = p[0]*q[2]

        R[0,2] = p[5]+p[4]
        R[2,0] = p[5]-p[4]

        p[4] = p[0]*q[3]
        p[5] = p[1]*q[2]

        R[1,2] = p[5]-p[4]
        R[2,1] = p[5]+p[4]

        return R

    def nav_eq(self, x, u, dt):
        g_t = np.array([0,0,-9.82]).reshape(3,1)
        f_t = np.matmul(self.q2dcm(x[6:10]),u[0:3])
        acc_t = f_t - g_t

        A = np.eye(6)
        A[0,3] = dt
        A[1,4] = dt
        A[2,5] = dt

        B = np.concatenate((np.eye(3)*0.5*pow(dt,2),np.eye(3)*dt))

        # Position and velocity prediction
        x[0:6] = np.matmul(A,x[0:6])+np.matmul(B,acc_t)

        # Attitude Quaternion
        w_tb = u[3:6]
        P = (w_tb[0]*dt)[0]
        Q = (w_tb[1]*dt)[0]
        R = (w_tb[2]*dt)[0]

        OMEGA = np.zeros((4,4))
        OMEGA[0,0:4] = 0.5*np.array([0, R, -Q, P])
        OMEGA[1,0:4] = 0.5*np.array([-R, 0, P, Q])
        OMEGA[2,0:4] = 0.5*np.array([Q, -P, 0, R])
        OMEGA[3,0:4] = 0.5*np.array([-P, -Q, -R, 0])

        v = np.linalg.norm(w_tb)*dt

        if v != 0:
            x[6:10] = np.matmul((cos(v/2)*np.eye(4) + 2/v*sin(v/2)*OMEGA),x[6:10])
        return x


    def state_space_model(self, x, u, Ts):
        Rb2t = self.q2dcm(x[6:10])
        f_t = np.matmul(Rb2t,u[0:3])
        St = np.array([[0, -f_t[2][0], f_t[1][0]],[f_t[2][0], 0, -f_t[0][0]],[-f_t[1][0], f_t[0][0], 0]])

        O = np.zeros((3,3))
        I = np.eye(3)
        Fc0 = np.concatenate((O,I,O,O,O), axis=1)
        Fc1 = np.concatenate((O,O,St,Rb2t,O), axis=1)
        Fc2 = np.concatenate((O,O,O,O,-Rb2t), axis=1)
        Fc3 = np.concatenate((O,O,O,O,O), axis=1)
        Fc4 = np.concatenate((O,O,O,O,O), axis=1)
        Fc = np.concatenate((Fc0, Fc1, Fc2, Fc3, Fc4))

        F = np.eye(15) + Ts*Fc
        G0 = np.concatenate((O,O,O,O),axis=1)
        G1 = np.concatenate((Rb2t,O,O,O),axis=1)
        G2 = np.concatenate((O,-Rb2t,O,O),axis=1)
        G3 = np.concatenate((O,O,I,O),axis=1)
        G4 = np.concatenate((O,O,O,I),axis=1)
        G = Ts*np.concatenate((G0,G1,G2,G3,G4))

        return (F,G)

    def get_Rb2p(self):
        # Function that returns the directional cosine matrix that relates the
        # body (IMU coordinate system) to the platform (vehicle coordinate system)
        # coordinate frame.
        return np.eye(3)

    def Gamma(self, q, epsilon):
        R = self.q2dcm(q)
        OMEGA = np.array([[0, -epsilon[2][0], epsilon[1][0]],[epsilon[2][0],0, -epsilon[0][0]],[-epsilon[1][0],epsilon[0][0],0]])
        R = np.matmul((np.eye(3)-OMEGA),R)
        q = self.dcm2q(R)
        return q


    def filter(self):
        Ts = self.dt
        # Calibrate the sensor measurements using current sensor bias estimate.
        self.u_h = self.u + self.delta_u_h
        # Update the INS navigation state
        self.xh = self.nav_eq(self.xh, self.u_h, Ts)
        # Get state space model matrices
        (self.F, self.G) = self.state_space_model(self.xh, self.u_h, Ts)
        # Time update of the Kalman filter state covariance.
        self.P = np.matmul(self.F,(np.matmul(self.P,np.transpose(self.F)))) + np.matmul(self.G, (np.matmul(block_diag(self.Q1, self.Q2),np.transpose(self.G))))
        # Defualt measurement observation matrix  and measurement covariance matrix
        y1 = self.y     # GPS data
        y2 = np.zeros((2,1))
        y = np.concatenate((y1,y2))

        Rn2p = np.matmul(self.get_Rb2p(),np.transpose(self.q2dcm(self.xh[6:10])))
        H1 = np.concatenate((np.eye(3),np.zeros((3,12))),axis=1)
        H2 = np.concatenate((np.zeros((3,3)), Rn2p, np.zeros((3,9))),axis=1)
        H = np.concatenate((H1, H2))

        R1 = np.concatenate((pow(self.sigma_gps,2)*np.eye(3), np.zeros((3,2))),axis=1)
        R2 = np.concatenate((np.zeros((2,3)), pow(self.sigma_non_holonomic,2)*np.eye(2)), axis=1)
        R = np.concatenate((R1,R2))

        tmp_H = np.zeros(15)
        tmp_y = []
        tmp_R = []
        for i in range(5):
            if self.data_idx[i] == 1:
                tmp_H = np.vstack((tmp_H,H[i,:]))
                tmp_y.append(y[i])
                tmp_R.append(R[i,i])
        self.data_idx[0:3] = 0


        H = tmp_H[1:]
        tmp_y = np.array(tmp_y)
        if len(tmp_y) == 5:
            self.gps_gt.append(tmp_y[0:3].flatten())
        y = np.array(tmp_y)
        R = np.eye(len(tmp_R))*np.array(tmp_R)

        # Calculate Kalman gain
        K = np.matmul(np.matmul(self.P,np.transpose(H)),np.linalg.inv(np.matmul(np.matmul(H,self.P),np.transpose(H)) + R))

        # Update the perturbation state estimate
        z = np.concatenate((np.zeros((9,1)),self.delta_u_h)) + np.matmul(K,(y-np.matmul(H[:,0:6],self.xh[0:6])))

        # Correct the navigation states using current perturbation estimates.
        self.xh[0:6] = self.xh[0:6] + z[0:6]
        self.xh[6:10] = self.Gamma(self.xh[6:10], z[6:9])
        self.delta_u_h = z[9:15]

        self.P = np.matmul((np.eye(15)-np.matmul(K,H)),self.P)
        self.xh_s.append(self.xh.flatten())

    def shutdown_handler(self):
        print("Shutting down and saving data!")
        self.xh_s = np.asarray(self.xh_s)
        self.gps_gt = np.asarray(self.gps_gt)
        self.imu_data = np.asarray(self.imu_data)
        np.savetxt("../data.csv", self.xh_s, delimiter=",")
        np.savetxt("../gps_data.csv", self.gps_gt, delimiter=",")
        np.savetxt("../imu_data.csv", self.imu_data, delimiter=",")


if __name__ == "__main__":
    rospy.init_node('sensor_fusion_node', anonymous=True)
    sf = SensorFusion()
    rospy.on_shutdown(sf.shutdown_handler)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        sf.filter()
        rate.sleep()
    rospy.spin()
    #print("Shutting down and saving data!")
    #sf.xh_s = np.asarray(sf.xh_s)
    #sf.gps_gt = np.asarray(sf.gps_gt)
    #sf.imu_data = np.asarray(sf.imu_data)
    #np.savetxt("../data.csv", sf.xh_s, delimiter=",")
    #np.savetxt("../gps_data.csv", sf.gps_gt, delimiter=",")
    #np.savetxt("../imu_data.csv", sf.imu_data, delimter=",")