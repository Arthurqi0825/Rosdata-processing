import pandas as pd
import rosbag 
import sympy as sp 
import numpy as np
from sympy import symbols, Matrix, sin, acos, sqrt, lambdify
import matplotlib.pyplot as plt
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

# quaternion to Rotation matrix, for pose calculation
def q2r(quaternion):#quaternion to rotation
    # quaternion = [w,x,y,z]
    q0, q1, q2, q3 = quaternion
    q0 = float(q0)
    q1 = float(q1)
    q2 = float(q2)
    q3 = float(q3)

    # print(type(q0))
    rotation_matrix = sp.Matrix([
        [1 - 2*q2*q2 - 2*q3*q3, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2],
        [2*q1*q2 + 2*q0*q3, 1 - 2*q1*q1 - 2*q3*q3, 2*q2*q3 - 2*q0*q1],
        [2*q1*q3 - 2*q0*q2, 2*q2*q3 + 2*q0*q1, 1 - 2*q1*q1 - 2*q2*q2]
    ])

    return rotation_matrix

def slerp_interpolation(q1, q2, target_frequency, current_frequency):
    # Calculate the interpolation ratio
    ratio = target_frequency / current_frequency

    # Create symbols for the quaternion components
    t = symbols('t')
    w1, x1, y1, z1 = symbols('w1 x1 y1 z1')
    w2, x2, y2, z2 = symbols('w2 x2 y2 z2')

    # Create symbolic quaternion expressions for Q1 and Q2
    Q1 = Matrix([w1, x1, y1, z1])
    Q2 = Matrix([w2, x2, y2, z2])

    # Calculate the angle between Q1 and Q2
    dot_product = Q1.dot(Q2)
    angle = acos(dot_product)

    # Calculate the interpolation weight factors
    weight_q1 = sin((1 - t) * angle) / sin(angle)
    weight_q2 = sin(t * angle) / sin(angle)

    # Perform Slerp interpolation
    interpolated_quaternion = Q1 * weight_q1 + Q2 * weight_q2

    # Create lambdified function for numerical evaluation
    variables = (t, w1, x1, y1, z1, w2, x2, y2, z2)
    interpolated_quaternion_func = lambdify(variables, interpolated_quaternion)

    return interpolated_quaternion_func

def convert_quaternions(quaternions, target_frequency, current_frequency):
    converted_quaternions = []
    slerp_func = slerp_interpolation(None, None, target_frequency, current_frequency)

    for i in range(len(quaternions) - 1):
        q1 = quaternions[i]
        q2 = quaternions[i + 1]

        # Perform numerical evaluation of the interpolated quaternion
        interpolated_quaternion = slerp_func(
             i / (len(quaternions) - 1), q1[0], q1[1], q1[2], q1[3], q2[0], q2[1], q2[2], q2[3]
        )

        converted_quaternions.append(interpolated_quaternion)

    return converted_quaternions
# Interpolating data 

bag = rosbag.Bag('/home/zhangziqi/Desktop/Rosdata/Rosdata-processing/test2.bag')

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
data_file = '/home/zhangziqi/Desktop/Rosdata/Rosdata-processing/test2.bag_RotorSpeed.txt'


# Read the data from the text and bag file
data = []

bag_file = '/home/zhangziqi/Desktop/Rosdata/Rosdata-processing/test2.bag'
topics = ['/Quad13/Dynamics/IMU', '/mavros/battery', '/mavros/vision_pose/pose']
for topic in topics:
    for _, msg, t in bag.read_messages(topics=[topic]):
        if topic == '/Quad13/Dynamics/IMU':
            ac_x.append(float(msg.acc_x))
            ac_y.append(float(msg.acc_y))
            ac_z.append(float(msg.acc_z))
        if topic == '/mavros/vision_pose/pose':
            qx.append(float(msg.pose.orientation.x))
            qy.append(float(msg.pose.orientation.y))
            qz.append(float(msg.pose.orientation.z))
            w.append(float(msg.pose.orientation.w))


# print(len(qx))
# print(len(inter_qx))
#  Rotor Speed Part:
# Parameters
div = 100*60/16384 #unit change of rotor speed
kv = 1.473e-8 # thrust parameters
g = -9.81
# Open file and convert into float
with open(data_file, 'r') as file:
    for line in file:
        values = line.strip().split(', ')
        data.append([(float(value)*div) for value in values])


# Extract the rotor speeds and calculate Ri^2 for each row
rotor1_speeds = [row[0] for row in data]
rotor2_speeds = [row[1] for row in data]
rotor3_speeds = [row[2] for row in data]
rotor4_speeds = [row[3] for row in data]
#  get the thrust data by Kv*ri^2
ri_squared = [r1**2 + r2**2 + r3**2 + r4**2 for r1, r2, r3, r4 in data]
r1_thrust = [kv* i**2 for i in rotor1_speeds]
r2_thrust = [kv* i**2 for i in rotor2_speeds]
r3_thrust = [kv* i**2 for i in rotor3_speeds]
r4_thrust = [kv* i**2 for i in rotor4_speeds]

total_thrust = [kv* i for i in ri_squared]

print(len(total_thrust))

quaternion_240hz = []

for i in range(len(qx)):
    # quaternion_240hz[i] = [w[i],qx[i],qy[i],qz[i]]
    # print([w[i],qx[i],qy[i],qz[i]])
    quaternion_240hz.append([w[i],qx[i],qy[i],qz[i]])
    # quaternion_240hz[i][0] = w[i]
    # quaternion_240hz[i][1] = qx[i]
    # quaternion_240hz[i][2] = qy[i]
    # quaternion_240hz[i][3] = qz[i]

target_frequency  = 100
current_frequency = 240
interred_quaternions = convert_quaternions(quaternion_240hz, target_frequency, current_frequency)

# for i in interred_quaternions:
#     print(i[0])
# get processed imu data in 100hz, which is much easy 
# since imu frequency is 1000hz
imu = [[],[],[]]
ite = 0
while(ite <= len(ac_x)):
    imu[0].append(ac_x[ite])
    imu[1].append(ac_y[ite])
    imu[2].append(ac_z[ite])
    ite += 10



# Mass calculation 
#  Calculating Method : m = (R*T)/(Ra+g)
#  T  = kv* Sum(Ri^2)
#  R : Rotational Matrix, Using a function to transfer Quaternion to Rotational matrix
#  a : imu data [ax, ay, az]
#  g : gravity vector [0,0, -9.81]


# 
mass = []
Th= []
for i in range(min(len(imu[0]),len(interred_quaternions),len(total_thrust))):
    #rotation: q2r
    qua = [interred_quaternions[i][0],interred_quaternions[i][1],interred_quaternions[i][2],interred_quaternions[i][3]]
    R = q2r(qua)
    
    G = sp.Matrix([0,0,-9.81]) # [0,0,-9.81].T
    A = sp.Matrix([imu[0][i],imu[1][i],imu[2][i]]) 
    T = sp.Matrix([0,0,total_thrust[i]])

    Thrust = R@T
    Acc    = R@A
    # Thrust = T
    # Acc    = A + G
    # Th.append(Thrust[2])
    mi =  Thrust[2]/Acc[2]
    # mi = (total_thrust[i])/(imu[2][i])
    # mi = (Thrust.dot(Acc))/(Acc.dot(Acc)) 
    mass.append(mi)


# timestamp = [i * 0.01 for i in range(len(mass))]
# # print(mass)
# plt.plot(timestamp,mass,color='b')
# plt.scatter(timestamp,mass, s= 20,color='r')
plt.plot(mass)
plt.show()
