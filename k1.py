from kinematic import *
from kinematic1 import *
import math
import ikpy
import numpy as np
from ikpy import plot_utils
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink,DHLink
# settheta = [0,0,0,0]
# trans = np.dot(goalpos(0,0,0,0),[[0],[0],[0],[1]])
# print(trans)
# basepoint =[trans[0][0],trans[1][0],trans[2][0]]
# settheta[0] = np.arctan2(basepoint[1], basepoint[0])
# settheta[0] = int (settheta[0]*180/PI)

# n = basepoint[0]**2 + basepoint[1]**2
# m = basepoint[2] - 65	
# n = np.sqrt(n)
# a = np.sqrt(m**2 + n**2)

# x = np.sqrt(65**2 + 83**2 -(2*65*83*np.cos((135/180.0)*PI)))
# temp = np.arccos((129**2-106**2+a**2)/(2*129*a))

# t1 = temp
# t2 = np.arctan2(m,n)
# print(m)
# print(n)
# print(x)

# print(t1)
# print(t2)


# settheta[1] = 90 - int ((t2 + t1)*180/PI) - 38


# belta = np.arccos((129**2 - a**2 + x**2)/(2*129*x))
# fi = np.arccos((65**2 - 83**2 + x**2)/(2*65*x))
# print(belta)
# print(fi)
# settheta[2] = 180 - int ((belta + fi)*180/PI) -47
# settheta[3] = -settheta[1]-settheta[2]-35
# print(settheta)
settheta = [0,0,0,0]
trans = goalpos1(0,0,0,0)
print(trans)
basepoint =[trans[0][0],trans[1][0],trans[2][0]]
settheta[0] = np.arctan2(basepoint[1], basepoint[0])
settheta[0] = int (settheta[0]*180/PI)

n = basepoint[0]**2 + basepoint[1]**2
m = basepoint[2] - 65	
n = np.sqrt(n)


a = np.sqrt(m**2 + n**2)
x = np.sqrt(a**2 + 83**2 - 2*a*83.0*(n/a))

temp = np.arccos((129**2 + 65**2 - x**2)/(2*129*65))

t3 = np.arccos((-129**2 + 65**2 + x**2)/(2*65*x))
t4 = np.arccos((83**2 - a**2 + x**2)/(2*83*x))
t1 = temp
t2 = np.arctan2(m,n)
print(m)
print(n)
print(x)

print(t1)
print(t2)


settheta[2] =  180 - int ((t1)*180/PI) -42
settheta[1] = 90 - 360 + int((t1+t3+t4)*180/PI) - 3
settheta[3] = 180 - int((t3+t4)*180/PI) - 90


print(settheta)

if settheta[2] >= 63:
	settheta[2] = 63
	s2 = 180 - 42 - 63
	s1 = 90 - settheta[1] - 120 - t2
	settheta[1] = settheta[1] + 120
	x1 = np.sqrt(a**2 + 129**2 - 2*a*129*np.cos(s1))
	x2 = np.arccos((a**2 + x1**2 - 129**2)/(2*a*x1))
	x3 = np.arccos((x1**2 + 83**2 - 65**2)/(2*83*x1))
	settheta[3] = 180 -(360 - s1 - s2 - x2 - x3) - 90
print(settheta)
(theta1,theta2,theta3,theta4) = (0,0.57,1.57,0)


mychain = Chain(name='neck', links=[
	#OriginLink(),
	URDFLink(
      name="shoulder",
      translation_vector=[0, 0, 0.065],
      orientation=[-0.5*PI, 0, theta1],
      rotation=[0, 0, 1],
    ),
	URDFLink(
       name="elbow",
     translation_vector = (np.transpose(np.dot(np.array([[np.cos(-PI*(156/180.0)+theta2), -np.sin(-PI*(156/180.0)+theta2), 0],
	[np.sin(-PI*(156/180.0)+theta2), np.cos(-PI*(156/180.0)+theta2), 0],
	[0, 0, 1]]),[[0.129],[0],[0]])))[0],
       orientation=[0, 0, -PI*(156/180.0)+theta2],
       rotation=[0, 0, 1],
     ),
	URDFLink(
     name="wrist",
     translation_vector=  (np.transpose(np.dot(np.array([[np.cos((99/180.0)*PI+theta3), -np.sin((99/180.0)*PI+theta3), 0],
	[np.sin((99/180.0)*PI+theta3), np.cos((99/180.0)*PI+theta3), 0],
	[0, 0, 1]]),[[0.065],[0],[0]])))[0],

     orientation=[0, 0, (99/180.0)*PI+theta3],
    rotation=[0, 0, 1],
     ),
	URDFLink(
	name="head",
     translation_vector=(np.transpose(np.dot(np.array([[np.cos((61/180.0)*PI+theta4), -np.sin((61/180.0)*PI+theta4), 0],
	[np.sin((61/180.0)*PI+theta4), np.cos((61/180.0)*PI+theta4), 0],
	[0, 0, 1]]),[[0.083],[0],[0]])))[0],

    orientation=[0, 0, (61/180.0)*PI+theta4],
    rotation=[0, 0, 1],
    )
 ])
print(mychain.forward_kinematics([0] * 4))
target_vector = [ -9.35651979e-02, 0, 4.69447774e-02]
target_frame = np.eye(4)
target_frame[:3, 3] = target_vector
print("The angles of each joints are : ", mychain.inverse_kinematics(target_frame))




