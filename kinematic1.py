import numpy as np
import math
PI = np.pi
#trans_mat = np.eye(4)
#end effect -51
def distance(a,b):
	x = a-b
	return np.sum(np.power(x,2))
def tmatrix1 (a,alpha,theta,d):
	temp = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
	[np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
	[0, np.sin(alpha), np.cos(alpha), d],
	[0, 0, 0, 1]])
	return temp
def goalpos1(theta1,theta2,theta3,theta4):
	trans_mat = np.eye(4)
	t1 = tmatrix1(0,-PI * 0.5,theta1,65)
	t2 = tmatrix1(129,0,-PI*(178/180.0)+theta2,0)
	t3 = tmatrix1(65,0,(135/180.0)*PI+theta3,0)
	t4 = tmatrix1(83,0,(39/180.0)*PI+theta4,0)
	trans_mat = np.dot(trans_mat,t1)
	#print(trans_mat)
	trans_mat = np.dot(trans_mat,t2)
	#print(trans_mat)
	trans_mat = np.dot(trans_mat,t3)
	#print(trans_mat)
	trans_mat = np.dot(trans_mat,t4)
	#print(trans_mat)
	return trans_mat
print(np.dot(goalpos1(0,(0/180)*PI,(0/180)*PI,(0/180)*PI),[[0],[0],[0],[1]]))