#!/usr/bin/env python
from imutils.video import VideoStream
import numpy as np
#import argparse
import cv2
import imutils

cap = cv2.VideoCapture('test.mp4') #change this

while(cap.isOpened()):
    ret, frame = cap.read()
    if frame is None:
        break

    frame = imutils.resize(frame, width=600)

    # kernel = np.array([[-1, -1, -1],
    #                [-1,  9, -1],
    #                [-1, -1, -1]])
    # sharpened = cv2.filter2D(frame, -1, kernel)

    processing = cv2.dilate(frame,None,iterations=3)
    processing = cv2.erode(processing,None,iterations=3)
    processing = cv2.erode(processing,None,iterations=3)
    processing = cv2.dilate(processing,None,iterations=3)

    gray = cv2.cvtColor(processing, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 50, param1=60, param2=40, minRadius=5, maxRadius=100)
#circles = cv2.HoughCircles(image, method, dp, minDist[, param1[, param2[, minRadius[, maxRadius]]]]])
#image: This is the input image (grayscale or BGR).
# method: This parameter defines the method for detecting circles. The most common method is cv2.HOUGH_GRADIENT, which uses a variant of the Hough Transform to detect circles in the image.
# dp: This is the inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1, the accumulator has the same resolution as the input image. If dp=2, the accumulator has half the resolution of the input image. It is a floating-point value.
# minDist: This parameter represents the minimum distance between the centers of detected circles. If the distance between the centers of two circles is less than this value, then one of the circles is eliminated. It is a floating-point value.
# param1: This is a gradient value used for edge detection. The higher the value, the more edges will be detected. It is a floating-point value.
# param2: This is a threshold for circle detection. A smaller value means more circles will be detected (including false circles), and a larger value means fewer circles (more stringent detection criteria). It is a floating-point value.
# minRadius: Minimum radius of the circles to be detected. It is an integer.
# maxRadius: Maximum radius of the circles to be detected. It is an integer.
    

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            cv2.circle(frame,(i[0],i[1]),i[2],(0,0,255),2)
            cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
        

    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()