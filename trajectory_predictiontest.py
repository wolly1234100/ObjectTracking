# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import math

#calculate degrees of angle between 3 points, with p2 as vertex
def angle(p1,p2,p3):
    p12 = math.sqrt(math.pow((p1[0]-p2[0]),2)+pow((p1[1]-p2[1]),2))
    p13 = math.sqrt(math.pow((p1[0]-p3[0]),2)+pow((p1[1]-p3[1]),2))
    p23 = math.sqrt(math.pow((p2[0]-p3[0]),2)+pow((p2[1]-p3[1]),2))

    angle = math.acos((math.pow(p12,2)+math.pow(p13,2)-math.pow(p23,2))/(2*p12*p13))
    return math.degrees(angle)

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the green duct tape and the puck
# ball in the HSV color space, then initialize the
# list of tracked points
greenDuctLower = (49, 82, 40)
greenDuctUpper = (104, 255, 255)
puckLower = (18, 151, 157)
puckUpper = (43, 255, 255)
pts = deque(maxlen=args["buffer"])
borderpts = []
minX = maxX = minY = maxY = 0
trajQ = deque(maxlen=2)

#grab the reference to the webcam
vs = VideoStream(src=0).start()

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
        angle1 = angle(borderpts[0],borderpts[1],borderpts[2])
        cv2.line(frame, borderpts[0], borderpts[1], (0,0,255), 2)
        cv2.line(frame, borderpts[2], borderpts[3], (0,0,255), 2)

        #display angle on screen
        cv2.putText(frame,str("%.2f"%angle1),borderpts[1],cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))

        borderpts = sorted(borderpts, key= lambda k:k[1])
        angle2 = angle(borderpts[0],borderpts[1],borderpts[2])
        cv2.line(frame, borderpts[0], borderpts[1], (0,0,255), 2)
        cv2.line(frame, borderpts[2], borderpts[3], (0,0,255), 2)

        #display angle on screen
        cv2.putText(frame,str("%.2f"%angle2),borderpts[1],cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))

        if (85 <= angle1 <= 95) and (85 <= angle2 <= 95):
            minX = max(borderpts[0][0],borderpts[2][0])
            maxX = min(borderpts[1][0],borderpts[3][0])
            minY = max(borderpts[0][1],borderpts[1][1])
            maxY = min(borderpts[2][1],borderpts[3][1])
            print("minX, maxX", minX, maxX)
            print("minY, maxY", minY, maxY)
            for x in range(len(borderpts)):
                #get the maximum limits of the boundary
                print(borderpts[x])
            break

        borderpts.clear()

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF


while True:
    # grab the current frame
    frame = vs.read()

    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the puck color
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, puckLower, puckUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    	cv2.CHAIN_APPROX_SIMPLE)

    #this line to is to make this code work with multiple versions of opencv
    cnts = imutils.grab_contours(cnts)
    center = None

    #display border outline defined by points
    #this will be the border outline of the hockey table
    borderpts.sort()
    cv2.line(frame, borderpts[0], borderpts[1], (0,0,255), 2)
    cv2.line(frame, borderpts[2], borderpts[3], (0,0,255), 2)
    borderpts = sorted(borderpts, key= lambda k:k[1])
    cv2.line(frame, borderpts[0], borderpts[1], (0,0,255), 2)
    cv2.line(frame, borderpts[2], borderpts[3], (0,0,255), 2)


    # only proceed if at least one contour was found
    if len(cnts) > 0:

        #display the circle and centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        #append the centroid for the frame to the queue
        trajQ.appendleft(center)

        # only proceed if the radius meets a minimum size
        if radius > 10:
        	# draw the circle and centroid on the frame,
        	# then update the list of tracked points
        	cv2.circle(frame, (int(x), int(y)), int(radius),
        		(0, 255, 255), 2)
        	cv2.circle(frame, center, 5, (0, 0, 255), -1)


        #if the centroid in the current frame is within the boundary,
        #display a line extrapolating the centroids in two consecutive frames
        if (minX <= trajQ[0][0] <= maxX) and (minY <= trajQ[0][1] <= maxY):

            #if there are two frames in the queue,
            #display a line extrapolating the centroids in two consecutive frames
            if len(trajQ) > 1:

                x2 = trajQ[0][0]
                y2 = trajQ[0][1]
                x1 = trajQ[1][0]
                y1 = trajQ[1][1]
                line_len = math.sqrt(math.pow((x1-x2),2)+pow((y1-y2),2))
                theta = math.atan2((y2-y1),x2-x1)
                x3 = x1 + line_len * 10 * math.cos(theta)
                y3 = y1 + line_len * 10 * math.sin(theta)
                cv2.line(frame, (int(x1),int(y1)), (int(x3),int(y3)), (0,0,255), 2)

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
