#Averaging Angles given three points /Users/Isaac/Desktop/SeniorDesign/ObjectTracking/AverageAngle.py
import math 

# init coordinates 
a = [1]
b = [1]
c = [1]
	
#Getting the sines and cosines 
#a
ax = math.degrees(math.cos((math.pi)))
ay = math.degrees(math.sin((math.pi)))
#b
bx = math.degrees(math.cos((math.pi)/2))
by = math.degrees(math.sin((math.pi)/2))
#c
cx = math.degrees(math.cos(0))
cy = math.degrees(math.sin(0))
	
#averaging x's 
xAv = (ax + bx + cx)
#averaging y's
yAv = (ay + by + cy)


#Getting Angle
theta = math.atan2(yAv,xAv)
#printing angle 
print(math.degrees(theta))