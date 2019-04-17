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
from border_calibration import calibrate
#import border_calibration as bcal

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
    #melissa added the v2_DOT_v3 and the if-else around the t1 and t2 calculations
    #when causing the program to crash by sending the puck through the baseline border, this function call around line 360 was printed in the traceback in the terminal
    #hypothesized the dot product was causing a division by zero
    # CODE STILL NOT FIXED
    v2_DOT_v3 = np.dot(v2, v3)
    #if v2_DOT_v3 > 0:
    t1 = np.cross(v2, v1) / v2_DOT_v3
    t2 = np.dot(v1, v3) / np.dot(v2, v3)
    #lse:
    #    print("puck isn't in the border")
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

#check if all elements in a list are the same
# http://stackoverflow.com/q/3844948/
def checkEqual(lst):
    return not lst or lst.count(lst[0]) == len(lst)

#Takes a que and compares the min and max items to see if puck is moving
def isMoving(queue):
    maxi = max(queue)
    mini = min(queue)
    ismoving = maxi[0] - mini[0]
    if ismoving > 2: #threshold of 2
        return True
    else:
         return False
#start serial connection with arduino
ser = serial.Serial('/dev/cu.usbmodem14101',9600)


#(strikerMaxY, strikerMinY) = calibrate(ser)
strikerMinY = 144
strikerMaxY = 370
strikerMinX = 305
strikerMaxX = 525

# define the lower and upper boundaries of the green duct tape and the puck
# all in the HSV color space, then initialize the
# list of tracked points
greenDuctLower = (47, 73, 0) #lower color range
greenDuctUpper = (70, 190, 255) #upper boundary for color
strikerLower = (3,167,121) #orange color
strikerUpper = (11,255,255) #orange color
puckLower = (163, 64, 84) #lower boundary of color range of puck (fuschia)
puckUpper = (172, 225, 255) #upper bound for puck color range
orangeLower = (0,219,113)
orangeUpper = (8,255,255)
purpleLower = (120,126,0)
purpleUpper = (133,255,255)
borderpts = []
minX = maxX = minY = maxY = 0
trajQ = deque(maxlen=3) #queue to store center coordinates to extrapolate trajectory line
crossbarQ = deque(maxlen=2) #queue to store crossbar points
intersectQ = deque(maxlen=2) #queue to store intersect points so we don't get stuck in the while loop
topLeft = btmLeft = topRight = btmRight = (0,0)


#grab the reference to the webcam
vs = VideoStream(src=0).start()

timeCount = timeStart = timeStop = elapsedTime = fps = 0
pt0 = pt1 = pt2 = pt3 = 0
ysteppuck = ystepstriker = None
stepper1Coord = stepper2Coord = None
fbStepper1Coord = fbStepper2Coord = None
crossbarIntersect = 0

# get the position of the four outside corners to define outside boundary of air hockey table
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
    # (x, y) center of the green object
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    	cv2.CHAIN_APPROX_SIMPLE)

    #this line to is to make this code work with multiple versions of opencv
    cnts = imutils.grab_contours(cnts) #grab contours
    center = None

    # only proceed if there are at least 4 contours found
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
            if radius > 10: #this is the striker
            	# draw the circle and centroid on the frame,
            	# then update the list of tracked points
            	cv2.circle(frame, (int(x), int(y)), int(radius),
            		(0, 255, 255), 2)
            	cv2.circle(frame, center, 5, (0, 0, 255), -1)

        #display border outline defined by points
        #this will be the border outline of the hockey table
        borderpts.sort() #sorts by the x

        leftpts = [borderpts[0],borderpts[1]]
        leftpts = sorted(leftpts, key= lambda k:k[1]) #sort left points by the y
        pt0 = leftpts[0] #top left coordinate
        pt1 = leftpts[1] #bottom left coordinate

        rightpts = [borderpts[2],borderpts[3]]
        rightpts = sorted(rightpts, key= lambda k:k[1])
        pt2 = rightpts[0] #top right coordinate
        pt3 = rightpts[1] #bottom right coordinate

        angle1 = angle(pt1,pt0,pt3) #calculate angles between the border points
        angle2 = angle(pt2,pt0,pt3) #first point listed is the vertex

        cv2.line(frame, pt0, pt1, (0,0,255), 2) #create the red line between border points
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

        if (85 <= angle1 <= 95) and (85 <= angle2 <= 95): #make sure angles are roughly 90degrees making the the shape rectangular
            borderpts = sorted(borderpts, key= lambda k:k[1])
            minY = max(borderpts[0][1],borderpts[1][1])
            maxY = min(borderpts[2][1],borderpts[3][1])
            borderpts.sort()
            minX = max(borderpts[0][0],borderpts[1][0])
            maxX = min(borderpts[2][0],borderpts[3][0])

            print("minX, maxX", minX, maxX)
            print("minY, maxY", minY, maxY)
            topLeft = (minX,minY) #lock in the coordinates
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

centerStriker = None

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

    # construct a mask for the color "purple", then perform
    maskStriker = cv2.inRange(hsv, purpleLower, purpleUpper)
    maskStriker = cv2.erode(maskStriker, None, iterations=2)
    maskStriker = cv2.dilate(maskStriker, None, iterations=2)

    # construct a mask for the color "orange", then perform
    maskCrossbar = cv2.inRange(hsv, orangeLower, orangeUpper)
    maskCrossbar = cv2.erode(maskCrossbar, None, iterations=2)
    maskCrossbar = cv2.dilate(maskCrossbar, None, iterations=2)

    # find contours in the puck mask and initialize the current
    # (x, y) center of the puck
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    	cv2.CHAIN_APPROX_SIMPLE)

    # find contours in the striker mask and initialize the current
    # (x, y) center of the striker
    cntsStriker = cv2.findContours(maskStriker.copy(), cv2.RETR_EXTERNAL,
    	cv2.CHAIN_APPROX_SIMPLE)

    # find contours in the crossbar mask and initialize the current
    # (x, y) center of the crossbar
    cntsCrossbar = cv2.findContours(maskCrossbar.copy(), cv2.RETR_EXTERNAL,
    	cv2.CHAIN_APPROX_SIMPLE)

    #this line to is to make this code work with multiple versions of opencv
    cnts = imutils.grab_contours(cnts)
    cntsStriker = imutils.grab_contours(cntsStriker)
    cntsCrossbar = imutils.grab_contours(cntsCrossbar)
    center = None
    centerCrossbar = None

    #display border outline defined by points
    #this will be the border outline of the hockey table
    borderpts.sort()
    cv2.line(frame, borderpts[0], borderpts[1], (0,0,255), 2)
    cv2.line(frame, borderpts[2], borderpts[3], (0,0,255), 2)
    borderpts = sorted(borderpts, key= lambda k:k[1])
    cv2.line(frame, borderpts[0], borderpts[1], (0,0,255), 2)
    cv2.line(frame, borderpts[2], borderpts[3], (0,0,255), 2)

    #draw a box around striking range area
    cv2.line(frame, (strikerMinX + 15, strikerMinY), (strikerMinX + 15, strikerMaxY), (0,0,255), 2)
    cv2.line(frame, (strikerMinX + 15, strikerMaxY), (strikerMaxX - 50, strikerMaxY), (0,0,255), 2)
    cv2.line(frame, (strikerMaxX - 50, strikerMaxY), (strikerMaxX - 50, strikerMinY), (0,0,255), 2)
    cv2.line(frame, (strikerMaxX - 50, strikerMinY), (strikerMinX + 15, strikerMinY), (0,0,255), 2)


    #display coordinates
    cv2.putText(frame,str(pt0),pt0,cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))
    cv2.putText(frame,str(pt1),pt1,cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))
    cv2.putText(frame,str(pt2),(pt2[0]-150,pt2[1]),cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))
    cv2.putText(frame,str(pt3),(pt3[0]-150,pt3[1]),cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))

    #display frames per second on screen
    cv2.putText(frame,"FPS: %.0f"%fps,(450,410),cv2.FONT_HERSHEY_PLAIN,2.0,(0,0,0))



    #track crossbar with
    if len(cntsCrossbar) > 0:

        cntsCrossbar = sorted(cntsCrossbar, key=cv2.contourArea)

        i = 0
        #diplay centroid of largest 4 contours
        for c in cntsCrossbar:
            #break loop after four iterations
            if i == 2:
                break
            i = i + 1
            #display the circle and centroid
            ((xCrossbar, yCrossbar), radiusCrossbar) = cv2.minEnclosingCircle(c)
            MCrossbar = cv2.moments(c)
            centerCrossbar = (int(MCrossbar["m10"] / MCrossbar["m00"]), int(MCrossbar["m01"] / MCrossbar["m00"]))

            crossbarQ.appendleft(centerCrossbar)

            # only proceed if the green radius meets a minimum size
            #if radiusCrossbar > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, centerCrossbar, 5, (0, 0, 255), -1)
            cv2.putText(frame,str(centerCrossbar),centerCrossbar,cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))

            if len(crossbarQ) > 1:
                cv2.line(frame, crossbarQ[0], crossbarQ[1], (0,0,255), 2)

    #draw enclosing circle around puck striker
    if len(cntsStriker) > 0:
        #display the circle and centroid
        cStriker = max(cntsStriker, key=cv2.contourArea)
        ((xStriker, yStriker), radiusStriker) = cv2.minEnclosingCircle(cStriker)
        MStriker = cv2.moments(cStriker)
        centerStriker = (int(MStriker["m10"] / MStriker["m00"]), int(MStriker["m01"] / MStriker["m00"]))

        # only proceed if the green radius meets a minimum size
        if radiusStriker > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xStriker), int(yStriker)), int(radiusStriker),
                (0, 255, 255), 2)
            cv2.circle(frame, centerStriker, 5, (0, 0, 255), -1)
            cv2.putText(frame,str(centerStriker),centerStriker,cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))

    # only proceed if at least one fuschia (puck) contour was found
    if len(cnts) > 0:

        #display the circle and centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        #append the centroid for the frame to the queue
        trajQ.appendleft(center)

        # draw the circle and centroid on the frame,
        # then update the list of tracked points
        cv2.circle(frame, (int(x), int(y)), int(radius),
        	(0, 255, 255), 2)
        cv2.circle(frame, center, 5, (0, 0, 255), -1)
        cv2.putText(frame,str(center),center,cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255))

        #if there are two frames in the queue,
        #display a line extrapolating the centroids in two consecutive frames
        if len(trajQ) > 2:
            x1 = trajQ[0][0] #most recent coordinate in queue
            y1 = trajQ[0][1] #most recent coordinate in queue
            x2 = trajQ[2][0]
            y2 = trajQ[2][1]
            line_len = math.sqrt(math.pow((x1-x2),2)+pow((y1-y2),2)) #this is the velocity of the puck
            theta = math.atan2((y1-y2),(x1-x2))

            #seeing if puck is moving
            if isMoving(trajQ) == False:
                #is the puck within striking range
                if ((strikerMinX + 15) < x1 < (strikerMaxX - 15)) and ((strikerMinY + 15) < y1 < (strikerMaxY - 15)):
                    '''
                    if not ((y1 - 5) <= centerStriker[1] <= (y1 + 5)) :
                        #translates opencv x,y coordinates to step,step coordinates
                        #move striker to puck's y value, striker's own x value
                        stepper1Coord = - translate(y1,strikerMinY,strikerMaxY, 0,2100) + (2050 - translate(centerStriker[0],strikerMinX,strikerMaxX, 0,2050))
                        stepper2Coord = - translate(y1,strikerMinY,strikerMaxY, 0,2100) - (2050 - translate(centerStriker[0],strikerMinX,strikerMaxX, 0,2050))
                    else:
                        #translates opencv x,y coordinates to step,step coordinates
                        stepper1Coord = - translate(y1,strikerMinY,strikerMaxY, 0,2100) + (2050 - translate((x1 - 10),strikerMinX,strikerMaxX, 0,2050))
                        stepper2Coord = - translate(y1,strikerMinY,strikerMaxY, 0,2100) - (2050 - translate((x1 - 10),strikerMinX,strikerMaxX, 0,2050))
                    '''
                    stepper1Coord = stepper2Coord = -1050

                #if not within striking range, return to defensive position
                else:
                    stepper1Coord = stepper2Coord = -1050

            #if the puck is moving
            else:
                #if the puck is moving towards robot end of the hockey table
                if x2 < x1:
                    #loop until the predicted puck trajectory intersects the baseline
                    while True:

                        #if the intersect queue contains items that are all the same, that means it is stuck in this while loop, so we break from it
                        intersectQ.appendleft((x1,y1))
                        if len(intersectQ) > 1:
                            if checkEqual(intersectQ):
                                break

                        #get intersection of current position and direction with the robot goal line (baseline)
                        intersectPoint = lineRayIntersectionPoint((x1,y1),(math.cos(theta),math.sin(theta)),topRight,btmRight)


                        #if the the puck is headed toward the baseline
                        #draw a line from the current centroid to the baseline, and break from the loop
                        if intersectPoint != []:
                            #intersect with baseline
                            (x3,y3) = intersectPoint[0]

                            #print trajectory of intercept point
                            cv2.line(frame, (int(x1),int(y1)), (int(x3),int(y3)), (0,0,255), 2)
                            cv2.circle(frame, (int(x3),int(y3)), 8, (0, 0, 255), -1)

                            '''
                            possible issue with missing crossbar line to fix
                            '''
                            #calculate intersection of trajectory with crossbar
                            crossbarIntersect = lineRayIntersectionPoint((x1,y1),(math.cos(theta),math.sin(theta)),crossbarQ[0],crossbarQ[1])

                            try:
                                cv2.circle(frame, (int(crossbarIntersect[0][0]),int(crossbarIntersect[0][1])), 8, (0, 0, 255), -1)

                                #map opencv coordinates to stepper coordinates
                                #the input range are just test values for now based on coordinates of corners
                                if 200 < int(y3) < 300:
                                    stepper1Coord = stepper2Coord = - translate(int(crossbarIntersect[0][1]),strikerMinY,strikerMaxY, 0,2100) #pucks predicted intercept with crossbar

                                else:
                                    stepper1Coord = stepper2Coord = - 1050

                            except:
                                '''
                                maybe use an old value for crossbarIntersect when an indexOutOfBounds exception is thrown
                                '''
                                pass

                            break

                        #if the puck headed towards one of the sidelines
                        else:
                            #if theta is negative, its heading towards the top sideline
                            if theta < 0:
                                print("theta < 0")
                                intersectPoint = lineRayIntersectionPoint((x1,y1),(math.cos(theta),math.sin(theta)),topLeft,topRight)
                                if intersectPoint != []:
                                    (x3,y3) = intersectPoint[0]
                                    cv2.line(frame, (int(x1),int(y1)), (int(x3),int(y3)), (0,0,255), 2)
                                    #set the current position equal to the end of the line and flip the angle
                                    (x1,y1) = (x3,y3)
                                    theta = -theta
                            #if theta is positive, its heading towards the bottom sideline
                            else:
                                print("theta > 0")
                                intersectPoint = lineRayIntersectionPoint((x1,y1),(math.cos(theta),math.sin(theta)),btmLeft,btmRight)
                                if intersectPoint != []:
                                    (x3,y3) = intersectPoint[0]
                                    cv2.line(frame, (int(x1),int(y1)), (int(x3),int(y3)), (0,0,255), 2)

                                    #set the current position equal to the end of the line and flip the angle
                                    (x1,y1) = (x3,y3)
                                    theta = -theta

                    #if the puck is moving away from the robot goal
                    else:
                        stepper1Coord = stepper2Coord = - 1050



            try:
                #translates opencv x,y coordinates to step,step coordinates
                fbStepper1Coord = - translate(centerStriker[1],strikerMinY,strikerMaxY, 0,2100) + (2050 - translate(centerStriker[0],strikerMinX,strikerMaxX, 0,2050))
                fbStepper2Coord = - translate(centerStriker[1],strikerMinY,strikerMaxY, 0,2100) - (2050 - translate(centerStriker[0],strikerMinX,strikerMaxX, 0,2050))

                #print(centerStriker)

                #encode puck intercept position
                stepper1Coord = str.encode(str(int(stepper1Coord)) + "\r")
                stepper2Coord = str.encode(str(int(stepper2Coord)) + "\t")

                #encode striker position for feedback loop
                fbStepper1Coord = str.encode(str(int(fbStepper1Coord)) + " ")
                fbStepper2Coord = str.encode(str(int(fbStepper2Coord)) + "\n")

                #combine the data to send it to the arduino
                data = stepper1Coord + stepper2Coord + fbStepper1Coord + fbStepper2Coord
                print("step1: ", int(stepper1Coord), "step2: ", int(stepper2Coord), "fbstep1: ", int(fbStepper1Coord), "fbstep2: ", int(fbStepper2Coord))
                ser.write(data) #write serial data
            except Exception as e:
                print(e)
                pass


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
