#!/usr/bin/env python

import numpy as np
from scipy.linalg import expm, logm
from pu2_header import *
import math

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for w1~6 and v1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))

	omegas = np.array([[0,0,1], #axes
			[0,1,0],
			[0,1,0],
			[0,1,0],
			[1,0,0],
			[0,1,0]])

	q = np.array([[-0.150,0.150,0.01], #displacements
		[-0.150,0.270,0.162],
		[0.094,0.270,0.162],
		[0.307,0.177,0.162],
		[0.307,0.260,0.162],
		[0.390,0.260,0.162]])

	v = np.array([[0.15,0.15,0], #v = -cross(omega,q)
		[-0.162,0,-0.15],
		[-0.162,0,0.094],
		[-0.162,0,0.307],
		[0,0.162,-0.26],
		[-0.162,0,0.390]])

	# global S
	S = [None, None, None, None, None, None]

	S[0] = np.array([[0, -omegas[0][2], omegas[0][1], v[0][0]],
				[omegas[0][2], 0, -omegas[0][0], v[0][1]],
				[-omegas[0][1], omegas[0][0], 0, v[0][2]],
				[0,0,0,0]])

	S[1] = np.array([[0, -omegas[1][2], omegas[1][1], v[1][0]],
				[omegas[1][2], 0, -omegas[1][0], v[1][1]],
				[-omegas[1][1], omegas[1][0], 0, v[1][2]],
				[0,0,0,0]])
	
	S[2] = np.array([[0, -omegas[2][2], omegas[2][1], v[2][0]],
				[omegas[2][2], 0, -omegas[2][0], v[2][1]],
				[-omegas[2][1], omegas[2][0], 0, v[2][2]],
				[0,0,0,0]])

	S[3] = np.array([[0, -omegas[3][2], omegas[3][1], v[3][0]],
				[omegas[3][2], 0, -omegas[3][0], v[3][1]],
				[-omegas[3][1], omegas[3][0], 0, v[3][2]],
				[0,0,0,0]])

	S[4] = np.array([[0, -omegas[4][2], omegas[4][1], v[4][0]],
				[omegas[4][2], 0, -omegas[4][0], v[4][1]],
				[-omegas[4][1], omegas[4][0], 0, v[4][2]],
				[0,0,0,0]])
	
	S[5] = np.array([[0, -omegas[5][2], omegas[5][1], v[5][0]],
				[omegas[5][2], 0, -omegas[5][0], v[5][1]],
				[-omegas[5][1], omegas[5][0], 0, v[5][2]],
				[0,0,0,0]])

	M = [[0, -1, 0, 0.390],
		[0, 0, -1, 0.401],
		[1, 0, 0, 0.2155],
		[0,0,0,1]]



	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()

	# T = expm(S[0]*theta1) @ expm(S[1]*theta2) @ expm(S[2]*theta3) @ expm(S[3]*theta4) @ expm(S[4]*theta5) @ expm(S[5]*theta6) @ M
	T = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(expm(S[0]*theta1),expm(S[1]*theta2)),expm(S[2]*theta3)),expm(S[3]*theta4)),expm(S[4]*theta5)),expm(S[5]*theta6)),M)
	
	#print(T)
	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	L1 = 0.152
	L2 = 0.120
	L3 = 0.244
	L4 = 0.093
	L5 = 0.213
	L6 = 0.083
	L7 = 0.083
	L8 = 0.082
	L9 = 0.0535
	L10 = 0.059
	yaw_WgripRad = math.radians(yaw_WgripDegree)
	
	#Inputs in UR3 Base Frame
	xgrip = xWgrip+0.15
	ygrip = yWgrip-0.15
	zgrip = zWgrip-0.01

	#Calculates center position of base plate
	xcen = xgrip-L9*math.cos(yaw_WgripRad)
	ycen = ygrip-L9*math.sin(yaw_WgripRad)
	zcen = zgrip

	#Calculates theta1
	Le = L2-L4+L6
	p = math.sqrt(xcen**2+ycen**2)
	theta1 = math.atan2(ycen,xcen)-math.asin(Le/p) #theta1

	#Calculates 3end position collinear with L6
	x3end = xcen-L7*math.cos(theta1)+(L6+0.027)*math.sin(theta1)
	y3end = ycen-L7*math.sin(theta1)-(L6+0.027)*math.cos(theta1)
	z3end = zcen+L10+L8

	theta6 = ((math.pi/2)-yaw_WgripRad)+theta1 #theta6

	#Calculates theta2,3,4 using law of cosines 
	x = math.sqrt(x3end**2+y3end**2)
	c = math.sqrt(x**2+(z3end-L1)**2)
	ep = math.acos((L3**2+L5**2-c**2)/(2*L3*L5))

	theta3 = math.pi-ep #theta3

	inb1 = math.asin((z3end-L1)/c)
	inb2 = math.asin(L5*math.sin(ep)/c)
	theta2 = -inb1-inb2 #theta2

	theta4 = -theta2-theta3 #theta4
	theta5 = -(math.pi/2) #theta5

	"""
	theta1 = 0.0
	theta2 = 0.0
	theta3 = 0.0
	theta4 = 0.0
	theta5 = 0.0
	theta6 = 0.0
	"""
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
