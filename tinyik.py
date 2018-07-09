from core import *
import numpy as ny
from kinematic1 import *
arm = Actuator(['z',[129.0,0.,0.], 'z',[65.0,0.,0.],'z',[83.0,0.,0.]])
arm.angles =[-PI*(156/180.0),(99/180.0)*PI,(61/180.0)*PI]
print(arm.ee)
settheta = [0,0,0,0]
trans = np.dot(goalpos1(0,1.57,-0.57,0),[[100],[0],[0],[1]])
print(trans)
basepoint =np.array([trans[0][0],trans[1][0],trans[2][0]])
settheta[0] = np.arctan2(basepoint[1], basepoint[0])
settheta[0] = int (settheta[0]*180/PI)

arm.angles =[-PI*(156/180.0)+1.57,(99/180.0)*PI-0.57,(61/180.0)*PI]
print(arm.ee)
#arm.ee = []
arm.ee=[basepoint[0],65-basepoint[2],basepoint[1]]
(settheta[1],settheta[2],settheta[3]) = np.round(np.rad2deg(arm.angles))
print(np.round(np.rad2deg(arm.angles)))

settheta[1] = settheta[1]+ 4 + 90
settheta[2] = settheta[2] - 38
settheta[3] = settheta[3] - 90
print(settheta)
print(arm.ee)
