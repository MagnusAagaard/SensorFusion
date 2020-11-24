#!/usr/bin/env python3
import sys
import os
import rospy
import mavros
from sensor_msgs.msg import NavSatFix, Imu
from mavros_msgs.msg import GPSRAW

import numpy as np
from scipy.linalg import block_diag
from math import pi, sqrt, atan2, cos, sin
import matplotlib.pyplot as plt

from utm import utmconv
import csv
from fbm import fbm


class SensorFusion:
    def __init__(self, matlab=False):
        self.uc = utmconv()
        #settings
        self.sigma_gps = 1/sqrt(3)
        self.sigma_non_holonomic = 20
        self.time_zero = rospy.Time.now()

        if not matlab:
            self.dt = 0.02
            self.y = np.zeros((3,1))
            self.data_idx = np.zeros(6, dtype=np.int8)
            self.data_idx[4:6] = 1
            msg = rospy.wait_for_message('mavros/imu/data_raw', Imu)
            self.u = np.asarray([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]).reshape(6,1)
            #self.u = np.zeros((6,1))
            self.xh = self.init_navigation_state()
            self.delta_u_h = np.zeros((6,1))
            (self.P, self.Q1, self.Q2, _, _) = self.init_filter()
            self.setup_subs()
            self.xh_s = []
            self.gps_gt_converted = []
            self.gps_noise_converted = []
            self.gps_gt = []
            self.imu_data = []
            self.gps_count = 0
        else:
            self.in_data = self.read_matlab_data()
            self.filter_matlab_data()
        
        
    def setup_subs(self):
        self.pos_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.pos_cb)
        #self.gps1_sub = rospy.Subscriber('/mavros/gpsstatus/gps1/raw', GPSRAW, self.gt_gps_callback)
        #self.gps2_sub = rospy.Subscriber('/mavros/gpsstatus/gps2/raw', GPSRAW, self.gps_callback)
        self.imu_sub = rospy.Subscriber('mavros/imu/data_raw', Imu, self.imu_cb)

    def gps_callback(self, msg):
        self.lat = msg.lat / pow(10,7)
        self.lon = msg.lon / pow(10,7)
        self.alt = msg.alt / 1000
        (hemisphere, zone, letter, e1, n1) = self.uc.geodetic_to_utm(self.lat,self.lon)
        print("Noise GPS: {},{}".format(n1,e1))
        self.data_idx[:3] = 1
        self.y[:3] = (np.asarray([[n1, e1, -self.alt]]).reshape(3,1)-self.initial_pos).reshape(3,1)
        self.gps_noise_converted.append(self.y[:3].flatten())
    
    def gt_gps_callback(self, msg):
        lat = msg.lat / pow(10,7)
        lon = msg.lon / pow(10,7)
        alt = msg.alt / 1000
        (hemisphere, zone, letter, e1, n1) = self.uc.geodetic_to_utm(lat,lon)
        y = np.zeros((3,1))
        print("GT: {},{}".format(n1,e1))
        #self.data_idx[:3] = 1
        y[:3] = (np.asarray([[n1, e1, -alt]]).reshape(3,1)-self.initial_pos).reshape(3,1)
        self.gps_gt_converted.append(y[:3].flatten())
    
    def pos_cb(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.alt = msg.altitude
        (hemisphere, zone, letter, e1, n1) = self.uc.geodetic_to_utm(self.lat,self.lon)
        if self.gps_count == 25:
            self.data_idx[:3] = 1
            self.gps_count = 0
            self.y = np.zeros((3,1))
            self.y[:3] = (np.asarray([[n1, e1, -msg.altitude]]).reshape(3,1)-self.initial_pos).reshape(3,1)
            y_gt = np.zeros((3,1))
            y_gt[:3] = (np.asarray([[n1, e1, -msg.altitude]]).reshape(3,1)-self.initial_pos).reshape(3,1)
            self.gps_gt_converted.append(y_gt[:3].flatten())
            self.gps_noise_converted.append(self.y[:3].flatten())
        else:
            self.gps_count += 1
        #self.data_idx[:3] = 1
        #self.y = np.zeros((3,1))
        #self.y[:3] = (np.asarray([[n1, e1, -msg.altitude]]).reshape(3,1)-self.initial_pos).reshape(3,1)
        #y_gt = np.zeros((3,1))
        #y_gt[:3] = (np.asarray([[n1, e1, -msg.altitude]]).reshape(3,1)-self.initial_pos).reshape(3,1)
        #self.gps_gt_converted.append(y_gt[:3].flatten())
        self.gps_gt.append([self.lat, self.lon, self.alt])

    def imu_cb(self, msg):
        
        self.u_acc = np.asarray([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]).reshape(3,1)
        self.u_gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]).reshape(3,1)
        
        #Ry = np.array([[cos(pi),0,sin(pi)],[0,1,0],[-sin(pi),0,cos(pi)]])
        Rx = np.array([[1,0,0],[0,cos(pi),-sin(pi)],[0,sin(pi),cos(pi)]])
        #Rz = np.array([[cos(pi/2), -sin(pi/2), 0],[sin(pi/2),cos(pi/2),0],[0,0,1]])
        self.u_acc = np.matmul(Rx,self.u_acc)
        self.u_gyro = np.matmul(Rx,self.u_gyro)
        self.u = np.concatenate((self.u_acc, self.u_gyro))
        self.imu_data.append(self.u.flatten())

    def init_navigation_state(self):
        roll = 0
        pitch = 0
        heading = pi/2
        # Init coord. rotational matrix
        Rb2t = np.transpose(np.array(self.Rt2b([roll, pitch, heading])))
        q = self.dcm2q(Rb2t)
        # Få inital pos estimate her og smid ind!
        msg = rospy.wait_for_message('/mavros/gpsstatus/gps2/raw', GPSRAW)
        self.lat = msg.lat / pow(10,7)
        self.lon = msg.lon / pow(10,7)
        self.alt = msg.alt / 1000
        #msg = rospy.wait_for_message('/mavros/global_position/global', NavSatFix)
        #self.lat = msg.latitude
        #self.lon = msg.longitude
        (hemisphere, zone, letter, e1, n1) = self.uc.geodetic_to_utm(self.lat,self.lon)
        self.initial_pos = np.array([n1, e1, -self.alt]).reshape(3,1)
        x_h = np.concatenate((np.zeros((6,1)), q))
        print(q)
        return x_h

    def Rt2b(self, ang):
        #function for calculation of the rotation matrix for rotaion from tangent frame to body frame.
        cr = cos(ang[0])
        sr = sin(ang[0])
        cp = cos(ang[1])
        sp = sin(ang[1])
        cy = cos(ang[2])
        sy = sin(ang[2])
        R = np.asarray([[cy*cp, sy*cp, -sp],[-sy*cr+cy*sp*sr, cy*cr+sy*sp*sr, cp*sr], [sy*sr+cy*sp*cr, -cy*sr+sy*sp*cr, cp*cr]])
        return R

    def dcm2q(self, R):
        # Function for transformation from directional cosine matrix to quaternions
        q = np.zeros((4,1))
        q[3] = 0.5*sqrt(1+np.sum(np.diag(R)))
        q[0] = (R[2,1]-R[1,2])/(4*q[3][0])
        q[1] = (R[0,2]-R[2,0])/(4*q[3][0])
        q[2] = (R[1,0]-R[0,1])/(4*q[3][0])
        return q

    def init_filter(self):
        P = np.zeros((15,15))
        # Initial Kalman filter uncertainties (standard deviations)
        P[0:3,0:3] = pow(10,2)*np.eye(3)    # Position = 10 [m]
        P[3:6,3:6] = pow(5,2)*np.eye(3)     # Velocity = 5 [m/s]
        factp = np.asarray([pow(1*pi/180,2), pow(1*pi/180,2), pow(20*pi/180,2)])
        P[6:9,6:9] = np.diag(factp)    # Attitude (roll, pitch, yaw) [rad]
        P[9:12,9:12] = pow(0.02,2)*np.eye(3)     # Accelerometer biases [m/s^2]
        P[12:15,12:15] = pow((0.05*pi/180),2)*np.eye(3) # Gyro biases [rad/s]
        # Process noise covariance
        Q1 = np.zeros((6,6))
        Q1[0:3,0:3] = np.diag(pow(0.05,2)*np.ones((3,3)))        # sigma acc
        Q1[3:6,3:6] = np.diag(pow(0.1*pi/180,2)*np.ones((3,3)))    # sigma gyro
        Q2 = np.zeros((6,6))
        Q2[0:3,0:3] = pow(0.0001,2)*np.eye(3)             # sigma acc bias
        Q2[3:,3:] = pow(0.01*pi/180,2)*np.eye(3)            # sigma gyro bias
        R = pow(3/sqrt(3),2)*np.eye(3)                      # GNSS-receiver position measurement noise
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

        B = np.concatenate((0.5*pow(dt,2)*np.eye(3),dt*np.eye(3)))

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

    def get_Rb2p(self, matlab=False):
        # Function that returns the directional cosine matrix that relates the
        # body (IMU coordinate system) to the platform (vehicle coordinate system)
        # coordinate frame.
        if matlab:
            return np.array([[0.9880, -0.1472, -0.0463], [0.1540, 0.9605, 0.2319], [0.0103, -0.2363, 0.9716]])
        return np.eye(3)

    def Gamma(self, q, epsilon):
        R = self.q2dcm(q)
        OMEGA = np.array([[0, -epsilon[2][0], epsilon[1][0]],[epsilon[2][0], 0, -epsilon[0][0]],[-epsilon[1][0],epsilon[0][0],0]])
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
        y2 = np.zeros((3,1))
        y = np.concatenate((y1,y2))

        Rn2p = np.matmul(self.get_Rb2p(),np.transpose(self.q2dcm(self.xh[6:10])))
        H1 = np.concatenate((np.eye(3),np.zeros((3,12))),axis=1)
        H2 = np.concatenate((np.zeros((3,3)), Rn2p, np.zeros((3,9))),axis=1)
        H = np.concatenate((H1, H2))

        R1 = np.concatenate((pow(self.sigma_gps,2)*np.eye(3), np.zeros((3,3))),axis=1)
        R2 = np.concatenate((np.zeros((1,3)), 0*np.eye(1) ,np.zeros((1,2))), axis=1)
        R3 = np.concatenate((np.zeros((2,4)), pow(self.sigma_non_holonomic,2)*np.eye(2)), axis=1)
        R = np.concatenate((R1,R2,R3))

        tmp_H = np.zeros(15)
        tmp_y = []
        tmp_R = []
        for i in range(6):
            if self.data_idx[i] == 1:
                tmp_H = np.vstack((tmp_H,H[i,:]))
                tmp_y.append(y[i])
                tmp_R.append(R[i,i])
        self.data_idx[0:3] = 0

        H = tmp_H[1:]
        tmp_y = np.array(tmp_y)
        #if len(tmp_y) == 5:
        #    self.gps_gt.append(tmp_y[0:3].flatten())
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

    def read_matlab_data(self):
        self.matlab_gps_data = []
        self.matlab_gps_t = []
        self.matlab_imu_t = []
        self.matlab_imu_acc = []
        self.matlab_imu_gyro = []
        print("Loading Matlab data!")
        with open('/home/magnus/CAS/SensorFusion/matlab_data/gps.csv') as csvfile:
            data_tmp = csv.reader(csvfile)
            for row in data_tmp:
                self.matlab_gps_data.append(row)
        with open('/home/magnus/CAS/SensorFusion/matlab_data/gps_t.csv') as csvfile:
            data_tmp = csv.reader(csvfile)
            for row in data_tmp:
                self.matlab_gps_t.append(row[0])
        with open('/home/magnus/CAS/SensorFusion/matlab_data/imu_acc.csv') as csvfile:
            data_tmp = csv.reader(csvfile)
            for row in data_tmp:
                self.matlab_imu_acc.append(row)
        with open('/home/magnus/CAS/SensorFusion/matlab_data/imu_gyro.csv') as csvfile:
            data_tmp = csv.reader(csvfile)
            for row in data_tmp:
                self.matlab_imu_gyro.append(row)
        with open('/home/magnus/CAS/SensorFusion/matlab_data/imu_t.csv') as csvfile:
            data_tmp = csv.reader(csvfile)
            for row in data_tmp:
                self.matlab_imu_t.append(row[0])

        self.matlab_gps_data = np.transpose(np.asarray(self.matlab_gps_data).astype(np.float))
        self.matlab_gps_t = np.asarray(self.matlab_gps_t).astype(np.float)
        self.matlab_imu_t = np.asarray(self.matlab_imu_t).astype(np.float)
        self.matlab_imu_acc = np.asarray(self.matlab_imu_acc).astype(np.float)
        self.matlab_imu_gyro = np.asarray(self.matlab_imu_gyro).astype(np.float)
        print("Done loading")

    def init_navigation_state_matlab(self, u):
        f = u[:,:100].mean(axis=1)
        roll = atan2(-f[1], -f[2])
        pitch = atan2(f[0], np.linalg.norm(f[1:3]))
        roll = 0
        pitch = 0
        heading = 320*pi/180
        # Init coord. rotational matrix
        Rb2t = np.transpose(np.array(self.Rt2b([roll, pitch, heading])))
        q = self.dcm2q(Rb2t)
        x_h = np.concatenate((np.zeros((6,1)), q))
        return x_h
    
    def filter_matlab_data(self):
        print("Running matlab filter")
        u = np.transpose(np.concatenate((self.matlab_imu_acc, self.matlab_imu_gyro), axis=1))
        t = self.matlab_imu_t
        x_h = self.init_navigation_state_matlab(u)
        delta_u_h = np.zeros((6,1))
        (P, Q1, Q2, _, _) = self.init_filter()
        N = np.size(u,axis=1)
        self.out_data_x_h = np.zeros((10,N))
        self.out_data_x_h[:,0] = x_h.flatten()
        self.out_data_diag_p = np.zeros((15,N))
        self.out_data_diag_p[:,0] = np.diag(P)
        self.out_data_delta_u_h = np.zeros((6,N))
        ctr_gnss_data=1
        ctr_speed_data=1
        for k in range(1,N):
            Ts = t[k] - t[k-1]
            u_h = u[:,k].reshape(6,1) + delta_u_h
            x_h = self.nav_eq(x_h,u_h,Ts)
            (F,G) = self.state_space_model(x_h, u_h, Ts)
            P = np.matmul(F,(np.matmul(P,np.transpose(F)))) + np.matmul(G, (np.matmul(block_diag(Q1, Q2),np.transpose(G))))
            y1 = self.matlab_gps_data[:,ctr_gnss_data].reshape(3,1)
            #y2 = in_data.SPEEDOMETER.speed(:,ctr_speed_data);
            y3 = np.zeros((3,1))
            y = np.concatenate((y1,y3))
            Rn2p = np.matmul(self.get_Rb2p(matlab=True),np.transpose(self.q2dcm(x_h[6:10])))
            H1 = np.concatenate((np.eye(3),np.zeros((3,12))),axis=1)
            H2 = np.concatenate((np.zeros((3,3)), Rn2p, np.zeros((3,9))),axis=1)
            H = np.concatenate((H1, H2))

            R1 = np.concatenate((pow(self.sigma_gps,2)*np.eye(3), np.zeros((3,3))),axis=1)
            R2 = np.concatenate((np.zeros((1,3)), 0*np.eye(1) ,np.zeros((1,2))), axis=1)
            R3 = np.concatenate((np.zeros((2,4)), pow(self.sigma_non_holonomic,2)*np.eye(2)), axis=1)
            R = np.concatenate((R1,R2,R3))

            ind = np.zeros(6)
            ind[4:] = 1

            if abs(t[k] - self.matlab_gps_t[ctr_gnss_data]) <= 0.005:
                ind[0:3] = 1
                ctr_gnss_data = min(ctr_gnss_data+1,len(self.matlab_gps_t)-1)

            tmp_H = np.zeros(15)
            tmp_y = []
            tmp_R = []
            for i in range(6):
                if ind[i] == 1:
                    tmp_H = np.vstack((tmp_H,H[i,:]))
                    tmp_y.append(y[i])
                    tmp_R.append(R[i,i])
            H = tmp_H[1:]
            tmp_y = np.array(tmp_y)
            y = np.array(tmp_y)
            R = np.eye(len(tmp_R))*np.array(tmp_R)
            K = np.matmul(np.matmul(P,np.transpose(H)),np.linalg.inv(np.matmul(np.matmul(H,P),np.transpose(H)) + R))
            z = np.concatenate((np.zeros((9,1)),delta_u_h)) + np.matmul(K,(y-np.matmul(H[:,0:6],x_h[0:6])))

            # Correct the navigation states using current perturbation estimates.
            x_h[0:6] = x_h[0:6] + z[0:6]
            x_h[6:10] = self.Gamma(x_h[6:10], z[6:9])
            delta_u_h = z[9:15]

            P = np.matmul((np.eye(15)-np.matmul(K,H)),P)
            self.out_data_x_h[:,k] = x_h.flatten()
            self.out_data_diag_p[:,k] = np.diag(P)
            self.out_data_delta_u_h[:,k] = delta_u_h.flatten()
        print("Done, saving data")
        np.savetxt("data.csv", np.transpose(self.out_data_x_h), delimiter=",")


    def shutdown_handler(self):
        print("Shutting down and saving data!")
        self.xh_s = np.asarray(self.xh_s)
        self.gps_gt = np.asarray(self.gps_gt)
        self.gps_gt_converted = np.asarray(self.gps_gt_converted)
        self.imu_data = np.asarray(self.imu_data)
        np.savetxt("../data.csv", self.xh_s, delimiter=",")
        #np.savetxt("../gps_data.csv", self.gps_gt, delimiter=",")
        np.savetxt("../imu_data.csv", self.imu_data, delimiter=",")
        np.savetxt("../gps_noise_converted.csv", self.gps_noise_converted, delimiter=",")
        np.savetxt("../gps_gt_converted.csv", self.gps_gt_converted, delimiter=",")


if __name__ == "__main__":

    rospy.init_node('sensor_fusion_node', anonymous=True)
    sf = SensorFusion(matlab=False)
    rospy.on_shutdown(sf.shutdown_handler)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        sf.filter()
        rate.sleep()
    rospy.spin()