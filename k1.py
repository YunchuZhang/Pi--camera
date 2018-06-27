from kinematic import *
import math

settheta = [0,0,0,0]
trans = np.dot(goalpos(0,0,0,0),[[50],[20],[0],[1]])
print(trans)
basepoint =[trans[0][0],trans[1][0],trans[2][0]]
settheta[0] = np.arctan2(basepoint[1], basepoint[0])
settheta[0] = int (settheta[0]*180/PI)

n = basepoint[0]**2 + basepoint[1]**2
m = basepoint[2] - 65	
n = np.sqrt(n)
a = np.sqrt(m**2 + n**2)

x = np.sqrt(65**2 + 83**2 -(2*65*83*np.cos((135/180.0)*PI)))
temp = np.arccos((129**2-106**2+a**2)/(2*129*a))
if math.isnan(temp):
	print("aaaaa")
	trans = np.dot(goalpos(0,0,0,0),[[30],[40],[-20],[1]])
	print(trans)
	basepoint =[trans[0][0],trans[1][0],trans[2][0]]
	settheta[0] = np.arctan2(basepoint[1], basepoint[0])
	settheta[0] = int (settheta[0]*180/PI)

	n = basepoint[0]**2 + basepoint[1]**2
	m = basepoint[2] - 65	
	n = np.sqrt(n)
	a = np.sqrt(m**2 + n**2)

	x = np.sqrt(65**2 + 83**2 -(2*65*83*np.cos((135/180.0)*PI)))
	t1 = np.arccos((129**2-106**2+a**2)/(2*129*a))
	if math.isnan(t1):
		print("bbbbb")
		trans = np.dot(goalpos(0,0,0,0),[[10],[60],[-40],[1]])
		print(trans)
		basepoint =[trans[0][0],trans[1][0],trans[2][0]]
		settheta[0] = np.arctan2(basepoint[1], basepoint[0])
		settheta[0] = int (settheta[0]*180/PI)

		n = basepoint[0]**2 + basepoint[1]**2
		m = basepoint[2] - 65	
		n = np.sqrt(n)
		a = np.sqrt(m**2 + n**2)

		x = np.sqrt(65**2 + 83**2 -(2*65*83*np.cos((135/180.0)*PI)))
		t1 = np.arccos((129**2-106**2+a**2)/(2*129*a))
		if math.isnan(t1):
			print("ccccc")
			trans = np.dot(goalpos(0,0,0,0),[[-80],[100],[0],[1]])
			print(trans)
			basepoint =[trans[0][0],trans[1][0],trans[2][0]]
			settheta[0] = np.arctan2(basepoint[1], basepoint[0])
			settheta[0] = int (settheta[0]*180/PI)

			n = basepoint[0]**2 + basepoint[1]**2
			m = basepoint[2] - 65	
			n = np.sqrt(n)
			a = np.sqrt(m**2 + n**2)

			x = np.sqrt(65**2 + 83**2 -(2*65*83*np.cos((135/180.0)*PI)))
			t1 = np.arccos((129**2-106**2+a**2)/(2*129*a))
		else:
			pass
	else:
		pass
else:
	t1 = temp
t2 = np.arctan2(m,n)
print(m)
print(n)
print(x)

print(t1)
print(t2)


settheta[1] = 90 - int ((t2 + t1)*180/PI) - 45


belta = np.arccos((129**2 - a**2 + x**2)/(2*129*x))
fi = np.arccos((65**2 - 83**2 + x**2)/(2*65*x))
print(belta)
print(fi)
settheta[2] = 180 - int ((belta + fi)*180/PI) -37
settheta[3] = -(180 - int ((belta + fi)*180/PI) + 90 - int ((t2 + t1)*180/PI))
print(settheta)
