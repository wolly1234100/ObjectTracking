# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import cv2
import imutils
import serial

#uses motors to determine maximum boundary of the reach of the striker
#param: serPort- a string with the name of the serial port for the arduino
#return: a list of maximum and minimum
def calibrate(serPort):
    #start serial connection with arduino
    ser = serPort #serial.Serial(serPort,9600)

    # define the lower and upper boundaries of the green duct tape and the puck
    # ball in the HSV color space, then initialize the
    # list of tracked points
    greenDuctLower = (47, 73, 0)
    greenDuctUpper = (70, 190, 255)
    calibrationQ = deque(maxlen=5)

    #grab the reference to the webcam
    vs = VideoStream(src=0).start()


    c = 0 #used to count frames
    step0y = step2100y = 0 #variables to store y coordinates of table extremities


    #move the puck to the 0 step position in the y direction
    while True:
        # grab the current frame
        frame = vs.read()

        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)


        # construct a mask for the color "green", then perform
        maskGreen = cv2.inRange(hsv, greenDuctLower, greenDuctUpper)
        maskGreen = cv2.erode(maskGreen, None, iterations=2)
        maskGreen = cv2.dilate(maskGreen, None, iterations=2)

        # find contours in the puck mask and initialize the current
        # (x, y) center of the ball
        cntsGreen = cv2.findContours(maskGreen.copy(), cv2.RETR_EXTERNAL,
        	cv2.CHAIN_APPROX_SIMPLE)

        #this line to is to make this code work with multiple versions of opencv
        cntsGreen = imutils.grab_contours(cntsGreen)
        centerGreen = None


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
                calibrationQ.appendleft(centerGreen[1])
                print(calibrationQ[0])

            if len(calibrationQ) > 1:
                if calibrationQ[0] > calibrationQ[1]:
                    print("moving left")
                    c = 0
                else:
                    print("no move")
                    c = c + 1
                    if c == 3:
                        ser.write(str.encode("!\n"))
                        step0y = calibrationQ[0]
                        break

        # show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break

        for i in range(1000000):
            pass

    c = 0

    print("First while loop done")
    movingFlag = 0 #flag to signal when the striker starts moving
    #move the puck to the 2100 step position in the y direction
    while True:
        # grab the current frame
        frame = vs.read()

        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)


        # construct a mask for the color "green", then perform
        maskGreen = cv2.inRange(hsv, greenDuctLower, greenDuctUpper)
        maskGreen = cv2.erode(maskGreen, None, iterations=2)
        maskGreen = cv2.dilate(maskGreen, None, iterations=2)

        # find contours in the puck mask and initialize the current
        # (x, y) center of the ball
        cntsGreen = cv2.findContours(maskGreen.copy(), cv2.RETR_EXTERNAL,
        	cv2.CHAIN_APPROX_SIMPLE)

        #this line to is to make this code work with multiple versions of opencv
        cntsGreen = imutils.grab_contours(cntsGreen)
        centerGreen = None


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
                calibrationQ.appendleft(centerGreen[1])


            if len(calibrationQ) > 4:
                if (calibrationQ[0] < calibrationQ[1]) and (calibrationQ[1] < calibrationQ[2]) and (calibrationQ[2] < calibrationQ[3]) and (calibrationQ[3] < calibrationQ[4]):
                    movingFlag = 1
                else:
                    if movingFlag == 0:
                        print("waiting")
                    else:
                        if calibrationQ[0] < calibrationQ[1]:
                            print("moving right")
                            c = 0
                        else:
                            print("no move")
                            c = c + 1

                            if c == 3:
                                ser.write(str.encode("?\n"))
                                step2100y = calibrationQ[0]
                                break


        # show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break

        for i in range(1000000):
            pass

    print("second while loop done")

    #stop the camera video stream
    vs.stop()

    # close all windows
    cv2.destroyAllWindows()

    return (step0y,step2100y)

#print(calibrate('/dev/cu.usbmodem1411'))
