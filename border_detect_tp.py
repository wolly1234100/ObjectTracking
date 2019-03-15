#no green box in the beginning
#predicts trajectory based on speed within confined of box


# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import math
import serial

#calculate degrees of angle between 3 points, with p1 as vertex
def angle(p1,p2,p3):
    p12 = math.sqrt(math.pow((p1[0]-p2[0]),2)+pow((p1[1]-p2[1]),2))
    p13 = math.sqrt(math.pow((p1[0]-p3[0]),2)+pow((p1[1]-p3[1]),2))
    p23 = math.sqrt(math.pow((p2[0]-p3[0]),2)+pow((p2[1]-p3[1]),2))

    angle = math.acos((math.pow(p12,2)+math.pow(p13,2)-math.pow(p23,2))/(2*p12*p13))
    return math.degrees(angle)

#the following three functions are thanks to Daniel Farrell via stackoverflow
#https://stackoverflow.com/questions/14307158/how-do-you-check-for-intersection-between-a-line-segment-and-a-line-ray-emanatin
def magnitude(vector):
   return np.sqrt(np.dot(np.array(vector),np.array(vector)))

def norm(vector):
    return np.array(vector)/magnitude(np.array(vector))

def lineRayIntersectionPoint(rayOrigin, rayDirection, point1, point2):
    # Convert to numpy arrays
    rayOrigin = np.array(rayOrigin, dtype=np.float)
    rayDirection = np.array(norm(rayDirection), dtype=np.float)
    point1 = np.array(point1, dtype=np.float)
    point2 = np.array(point2, dtype=np.float)

    # Ray-Line Segment Intersection Test in 2D
    # http://bit.ly/1CoxdrG
    v1 = rayOrigin - point1
    v2 = point2 - point1
    v3 = np.array([-rayDirection[1], rayDirection[0]])
    t1 = np.cross(v2, v1) / np.dot(v2, v3)
    t2 = np.dot(v1, v3) / np.dot(v2, v3)
    if t1 >= 0.0 and t2 >= 0.0 and t2 <= 1.0:
        return [rayOrigin + t1 * rayDirection]
    return []

#returns a value mapped from one range to another
#left is input range, right is output range
def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

#start serial connection with arduino
ser = serial.Serial('/dev/cu.usbmodem1411',9600)

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the green duct tape and the puck
# ball in the HSV color space, then initialize the
# list of tracked points
greenDuctLower = (47, 73, 0)
greenDuctUpper = (70, 190, 255)
puckLower = (163, 64, 84)
puckUpper = (172, 225, 255)
pts = deque(maxlen=args["buffer"])
borderpts = []
minX = maxX = minY = maxY = 0
trajQ = deque(maxlen=4)
topLeft = btmLeft = topRight = btmRight = (0,0)


#grab the reference to the webcam
vs = VideoStream(src=0).start()

timeCount = timeStart = timeStop = elapsedTime = fps = 0
pt0 = pt1 = pt2 = pt3 = 0

# keep looping
while True:
    # grab the current frame
    frame = vs.read()

    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenDuctLower, greenDuctUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    	cv2.CHAIN_APPROX_SIMPLE)

    #this line to is to make this code work with multiple versions of opencv
    cnts = imutils.grab_contours(cnts)
    center = None

    # only proceed if at least one contour was found
    if len(cnts) >= 4:
    	# sort contours to find largest contours
        cnts = sorted(cnts, key=cv2.contourArea)

        i = 0
        #diplay centroid of largest 4 contours
        for c in cnts:
            #break loop after four iterations
            if i == 4:
                break
            i = i + 1

            #get minimum enclosing circle and centroid
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            #add the four centroids to the queue
            borderpts.append(center)

            # only proceed if the radius meets a minimum size
            if radius > 10:
            	# draw the circle and centroid on the frame,
            	# then update the list of tracked points
            	cv2.circle(frame, (int(x), int(y)), int(radius),
            		(0, 255, 255), 2)
            	cv2.circle(frame, center, 5, (0, 0, 255), -1)

        #display border outline defined by points
        #this will be the border outline of the hockey table
        borderpts.sort()

        leftpts = [borderpts[0],borderpts[1]]
        leftpts = sorted(leftpts, key= lambda k:k[1])
        pt0 = leftpts[0] #top left coordinate
        pt1 = leftpts[1] #bottom left coordinate

        rightpts = [borderpts[2],borderpts[3]]
        rightpts = sorted(rightpts, key= lambda k:k[1])
        pt2 = rightpts[0] #top right coordinate
        pt3 = rightpts[1] #bottom right coordinate

        angle1 = angle(pt1,pt0,pt3)
        angle2 = angle(pt2,pt0,pt3)

        cv2.line(frame, pt0, pt1, (0,0,255), 2)
        cv2.line(frame, pt1, pt3, (0,0,255), 2)
        cv2.line(frame, pt3, pt2, (0,0,255), 2)
        cv2.line(frame, pt2, pt0, (0,0,255), 2)

        #display angle on screen
        cv2.putText(frame,str("%.2f"%angle1),(pt1[0],pt1[1]+50),cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))
        cv2.putText(frame,str("%.2f"%angle2),(pt2[0]-150,pt2[1]+50),cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))

        #display coordinates
        cv2.putText(frame,"topLeft",pt0,cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))
        cv2.putText(frame,"btmLeft",pt1,cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))
        cv2.putText(frame,"topRight",(pt2[0]-150,pt2[1]),cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))
        cv2.putText(frame,"btmRight",(pt3[0]-150,pt3[1]),cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))

        if (85 <= angle1 <= 95) and (85 <= angle2 <= 95):
            borderpts = sorted(borderpts, key= lambda k:k[1])
            minY = max(borderpts[0][1],borderpts[1][1])
            maxY = min(borderpts[2][1],borderpts[3][1])
            borderpts.sort()
            minX = max(borderpts[0][0],borderpts[1][0])
            maxX = min(borderpts[2][0],borderpts[3][0])

            print("minX, maxX", minX, maxX)
            print("minY, maxY", minY, maxY)
            topLeft = (minX,minY)
            btmLeft = (minX,maxY)
            topRight = (maxX,minY)
            btmRight = (maxX,maxY)

            break

        borderpts.clear()

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

while True:
    # grab the current frame
    frame = vs.read()

    #calculate fps
    if timeCount == 0:
        timeStart = time.time()

    timeCount = timeCount + 1

    if timeCount == 20:
        timeStop = time.time()
        elapsedTime = timeStop - timeStart
        fps = 20/elapsedTime
        timeCount = 0

    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the puck color
    mask = cv2.inRange(hsv, puckLower, puckUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # construct a mask for the color "green", then perform
    maskGreen = cv2.inRange(hsv, greenDuctLower, greenDuctUpper)
    maskGreen = cv2.erode(maskGreen, None, iterations=2)
    maskGreen = cv2.dilate(maskGreen, None, iterations=2)

    # find contours in the puck mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    	cv2.CHAIN_APPROX_SIMPLE)

    # find contours in the puck mask and initialize the current
    # (x, y) center of the ball
    cntsGreen = cv2.findContours(maskGreen.copy(), cv2.RETR_EXTERNAL,
    	cv2.CHAIN_APPROX_SIMPLE)

    #this line to is to make this code work with multiple versions of opencv
    cnts = imutils.grab_contours(cnts)
    cntsGreen = imutils.grab_contours(cntsGreen)
    center = None
    centerGreen = None

    #display border outline defined by points
    #this will be the border outline of the hockey table

    borderpts.sort()
    cv2.line(frame, borderpts[0], borderpts[1], (0,0,255), 2)
    cv2.line(frame, borderpts[2], borderpts[3], (0,0,255), 2)
    borderpts = sorted(borderpts, key= lambda k:k[1])
    cv2.line(frame, borderpts[0], borderpts[1], (0,0,255), 2)
    cv2.line(frame, borderpts[2], borderpts[3], (0,0,255), 2)

    #display coordinates
    cv2.putText(frame,str(pt0),pt0,cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))
    cv2.putText(frame,str(pt1),pt1,cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))
    cv2.putText(frame,str(pt2),(pt2[0]-150,pt2[1]),cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))
    cv2.putText(frame,str(pt3),(pt3[0]-150,pt3[1]),cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))



    #display frames per second on screen
    cv2.putText(frame,"FPS: %.0f"%fps,(450,410),cv2.FONT_HERSHEY_PLAIN,2.0,(0,0,0))


    #draw enclosing circle around green puck striker
    if len(cntsGreen) > 0:
        #display the circle and centroid
        cGreen = max(cntsGreen, key=cv2.contourArea)
        ((xGreen, yGreen), radiusGreen) = cv2.minEnclosingCircle(cGreen)
        MGreen = cv2.moments(cGreen)
        centerGreen = (int(MGreen["m10"] / MGreen["m00"]), int(MGreen["m01"] / MGreen["m00"]))

        # only proceed if the green radius meets a minimum size
        if radiusGreen > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xGreen), int(yGreen)), int(radiusGreen),
                (0, 255, 255), 2)
            cv2.circle(frame, centerGreen, 5, (0, 0, 255), -1)
            cv2.putText(frame,str(centerGreen),centerGreen,cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))


    # only proceed if at least one contour was found
    if len(cnts) > 0:

        #display the circle and centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        #append the centroid for the frame to the queue
        trajQ.appendleft(center)


        #if the centroid in the current frame is within the boundary,
        #extrapolate trajectory
        if (minX <= trajQ[0][0] <= maxX) and (minY <= trajQ[0][1] <= maxY):

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                	(0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                cv2.putText(frame,str(center),center,cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))

            #if there are two frames in the queue,
            #display a line extrapolating the centroids in two consecutive frames
            if len(trajQ) > 3:
                x1 = trajQ[0][0]
                y1 = trajQ[0][1]
                x2 = trajQ[3][0]
                y2 = trajQ[3][1]
                line_len = math.sqrt(math.pow((x1-x2),2)+pow((y1-y2),2))
                theta = math.atan2((y1-y2),(x1-x2))

                #if the object is moving towards one end of the hockey table
                if x1 < x2:

                    #loop until the predicted puck trajectory intersects the baseline
                    while True:
                        intersectPoint = lineRayIntersectionPoint((x1,y1),(math.cos(theta),math.sin(theta)),topLeft,btmLeft)

                        #if the the puck is headed toward the baseline
                        #draw a line from the current centroid to the baseline, and break from the loop
                        if intersectPoint != []:
                            (x3,y3) = intersectPoint[0]

                            #map opencv coordinates to stepper coordinates
                            #the input range are just test values for now based on coordinates of corners
                            #we have to add a calibration switch to tell where the actual limits are
                            ysteppuck = translate(int(y3),52,278,0,2100) #puck's predicted y coordinate
                            ystepstriker = translate(int(centerGreen[1]),47,280,0,2100) #striker's current y coordinate

                            #just for test, serial ouput it every 9 frames
                            #if timeCount == 9:

                            #change float to int, change int to string, append newline character, change string from unicode to 'byte'
                            ystepstriker = str.encode(str(int(ystepstriker)) + "\r")
                            ysteppuck = str.encode(str(int(ysteppuck)) + "\n")
                            data = ystepstriker + ysteppuck
                            ser.write(data) #write serial data

                            cv2.line(frame, (int(x1),int(y1)), (int(x3),int(y3)), (0,0,255), 2)
                            cv2.circle(frame, (int(x3),int(y3)), 8, (0, 0, 255), -1)

                            break

                        #if the puck headed towards one of the sidelines
                        else:
                            #if theta is negative, its heading towards the top sideline
                            if theta < 0:
                                intersectPoint = lineRayIntersectionPoint((x1,y1),(math.cos(theta),math.sin(theta)),topLeft,topRight)
                                if intersectPoint != []:
                                    (x3,y3) = intersectPoint[0]
                                    cv2.line(frame, (int(x1),int(y1)), (int(x3),int(y3)), (0,0,255), 2)

                                    #set the current position equal to the end of the line and flip the angle
                                    (x1,y1) = (x3,y3)
                                    theta = -theta
                            #if theta is positive, its heading towards the bottom sideline
                            else:
                                intersectPoint = lineRayIntersectionPoint((x1,y1),(math.cos(theta),math.sin(theta)),btmLeft,btmRight)
                                if intersectPoint != []:
                                    (x3,y3) = intersectPoint[0]
                                    cv2.line(frame, (int(x1),int(y1)), (int(x3),int(y3)), (0,0,255), 2)

                                    #set the current position equal to the end of the line and flip the angle
                                    (x1,y1) = (x3,y3)
                                    theta = -theta


    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

#stop the camera video stream
vs.stop()

# close all windows
cv2.destroyAllWindows()
