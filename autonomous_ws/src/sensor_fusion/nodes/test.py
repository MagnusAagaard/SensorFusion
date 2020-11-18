import numpy as np
from math import sqrt, pi
R = np.asarray([[1,1,1],[2,2,2],[3,3,3]])
print(R)
q4 = 0.5*sqrt(1+np.sum(np.diag(R)))
print(q4)
q1 = (R[2,1]-R[1,2])/(4*q4)
print(q1)
q = np.zeros((4,1))
q[0] = 0.1
q[1] = 0.2
q[2] = 0.3
q[3] = 0.4
zeros = np.zeros((6,1))
x_h = np.concatenate((np.zeros((6,1)), q))
print(x_h)
P = np.zeros((15,15))
P[0:3,0:3] = pow(10,2)*np.eye(3)
factp = np.asarray([pow(1*pi/180,2), pow(1*pi/180,2), pow(20*pi/180,2)])
print(factp)
P[6:9,6:9] = np.diag(factp)

print(np.diag(pow(0.05,2)*np.eye(3)))
Q1 = np.zeros((6,6))
Q1[0:3,0:3] = np.diag(pow(0.05,2)*np.eye(3))
print(Q1)
Q2 = np.zeros((6,6))
Q2[0:3,0:3] = np.eye(3)* pow(0.0001,2)
print(Q2)

H = np.concatenate((np.eye(3), np.zeros((3,12))), axis=1)
print(H)

q = 2*np.ones((4,1))
p = np.zeros((6,1))
p[0:3] = pow(q[0:3],2)

B = np.concatenate((np.eye(3)*0.5*pow(0.02,2),np.eye(3)*0.02))
print(B)

OMEGA = np.zeros((4,4))
OMEGA[0,0:4] = 0.5*np.array([0,1,2,3])

data_idx = np.zeros(5)
print(data_idx)

data_idx[3:5] = 1
print(data_idx)
H = np.eye(5)
print(H[1,:])
tmp_H = np.zeros(5)
for i in range(5):
    if data_idx[i] == 1:
        tmp_H = np.vstack((tmp_H,H[i,:]))

print(tmp_H[1:])

print(np.eye(2)*np.array([2,4]))