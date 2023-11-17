import pandas as pd
import rosbag 
import sympy as sp 
import numpy as np 
import matplotlib as plt
import math
# 1, Using List to get all Quaternion
# 2. By Interpolating data, Transfer the 240HZ pose to 100 HZ pose--> which Interpolatoing: Lagrange ?
# 3, Get the data from IMU, From 1000 HZ to 100 HZ--> 1+10i
# 4, Get the data from Rotor Speed, Original 100 HZ --> 1+i 

#  Regrouping the data into list

#  Calculating Method : m = (R*T)/(Ra+g)
#  T  = kv* Sum(Ri^2)
#  R : Rotational Matrix, Using a function to transfer Quaternion to Rotational matrix
#  a : imu data [ax, ay, az]
#  g : gravity vector [0,0, -9.81]
# Read the data from the file
bag = rosbag.Bag('/home/zhangziqi/Desktop/Rosdata/test2.bag')

topics = ['/Quad13/Dynamics/IMU', '/mavros/battery', '/mavros/vision_pose/pose']

# Quaternion List
qx = []
qy = []
qz = []
w = []

# IMU Acceleration
ac_x = []
ac_y = []
ac_z = []

# rotorspeed, 
# Specify the path to your exported data text file
data_file = '/home/zhangziqi/Desktop/Rosdata/test2.bag_RotorSpeed.txt'


# Read the data from the text file
data = []

bag_file = '/home/zhangziqi/Desktop/Rosdata/test2.bag'
topics = ['/Quad13/Dynamics/IMU', '/mavros/battery', '/mavros/vision_pose/pose']
for topic in topics:
    for _, msg, t in bag.read_messages(topics=[topic]):
        if topic == '/Quad13/Dynamics/IMU':
            ac_x.append(float(msg.acc_x))
            ac_y.append(float(msg.acc_y))
            ac_z.append(float(msg.acc_z))
            
print(type(ac_x[300]))


#  Rotor Speed Part:
# Parameters
# div = 100*60/16384 #unit change of rotor speed
# kv = 1.473e-8 # thrust perameters
# g = -9.81
# # Open file and convert into float
# with open(data_file, 'r') as file:
#     for line in file:
#         values = line.strip().split(', ')
#         data.append([(float(value)*div) for value in values])


# # Extract the rotor speeds and calculate Ri^2 for each row
# rotor1_speeds = [row[0] for row in data]
# rotor2_speeds = [row[1] for row in data]
# rotor3_speeds = [row[2] for row in data]
# rotor4_speeds = [row[3] for row in data]
# #  get the thrust data by Kv*ri^2
# ri_squared = [r1**2 + r2**2 + r3**2 + r4**2 for r1, r2, r3, r4 in data]
# r1_thrust = [kv* i**2 for i in rotor1_speeds]
# r2_thrust = [kv* i**2 for i in rotor2_speeds]
# r3_thrust = [kv* i**2 for i in rotor3_speeds]
# r4_thrust = [kv* i**2 for i in rotor4_speeds]

# total_thrust = [kv* i for i in ri_squared]
