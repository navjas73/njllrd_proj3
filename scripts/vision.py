#!/usr/bin/env python

import roslib
import rospy
import numpy as np
import cv2
import cv2.cv as cv
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from njllrd_proj3.srv import *
from njllrd_proj3.msg import *


class image_blur:

  def __init__(self):
    # initialize a node called imaging
    rospy.init_node("imaging")

    # Create publisher
    self.pub = rospy.Publisher('ball_position',ball, queue_size=10)
    # create a window to display results in
    cv2.namedWindow("image_view", 1)

    # subscribe to proper topic
    self.image_sub = rospy.Subscriber("camera/rgb/image_color", Image, self.callback_blocks)

  def callback(self,data):
    """ This is a callback which recieves images and processes them. """
    # convert image into openCV format
    bridge = CvBridge()
    try:
      # bgr8 is the pixel encoding -- 8 bits per color, organized as blue/green/red
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      # all print statements should use a rospy.log_ form, don't print!
      rospy.loginfo("Conversion failed")

    # we could do anything we want with the image here
    # for now, we'll blur using a median blur


    cv2.imshow("rgb", cv_image)

    imgHSV = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
    cv2.imshow("image_view", imgHSV)

    imgHSV = cv2.medianBlur(imgHSV,3)
    cv2.imshow("image_view_blur1", imgHSV)

    #hsv_min = np.array([150,100,70])
    #hsv_max = np.array([255,255,255])
    hsv_min = np.array([150,100,70])
    hsv_max = np.array([220,255,255])

    img_thr = cv2.inRange(imgHSV,hsv_min,hsv_max)
    cv2.imshow("image_thr", img_thr)

    img_thr = cv2.medianBlur(img_thr,5)
    cv2.imshow("image_thr_blur", img_thr)
    '''
    image_copy = cv_image.copy()
    circles = cv2.HoughCircles(img_thr, cv.CV_HOUGH_GRADIENT, 1, 10, param1 = 100, param2 = 20)
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(image_copy, (x, y), r, (0, 255, 0), 2)
            cv2.rectangle(image_copy, (x - 1, y - 1), (x + 1, y + 1), (0, 128, 255), -1)
             
            # show the output image
        cv2.imshow("output", np.hstack([cv_image, image_copy]))
    '''

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
     
    params.filterByColor = True
    params.blobColor = 255
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 100
    params.maxArea = 200
     
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.5
     
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.87
     
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.5
     
    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
    else :
        detector = cv2.SimpleBlobDetector_create(params)

    keypoints = detector.detect(img_thr)

    if keypoints:
        x = keypoints[0].pt[0]
        y = keypoints[0].pt[1]
        t = rospy.get_time()
        self.pub.publish(x = x, y = y, t = t)

        im_with_keypoints = cv2.drawKeypoints(img_thr, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.imshow("Keypoints", im_with_keypoints)
    
    '''
    cv2.cvtColor(cv_image, cv_image_gray, cv2.COLOR_RGB2GRAY);
    
    ret,th1 = cv2.threshold(cv_image_gray,127,255,cv2.THRESH_BINARY)
    th2 = cv2.adaptiveThreshold(cv_image_gray,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
                cv2.THRESH_BINARY,11,2)
    th3 = cv2.adaptiveThreshold(cv_image_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                cv2.THRESH_BINARY,11,2)

    titles = ['Original Image', 'Global Thresholding (v = 127)',
                'Adaptive Mean Thresholding', 'Adaptive Gaussian Thresholding']
    images = [cv_image_gray, th1, th2, th3]
    
    for i in xrange(4):
        plt.subplot(2,2,i+1),plt.imshow(images[i],'gray')
        plt.title(titles[i])
        plt.xticks([]),plt.yticks([])
    plt.show()
    '''
    #show the image
    #cv2.imshow("image_view", imgHSV)
    #cv2.imshow("image_view2", cv_image)
    cv2.waitKey(3)
	

  def callback_blocks(self,data):
    """ This is a callback which recieves images and processes them. """
    # convert image into openCV format
    bridge = CvBridge()
    try:
      # bgr8 is the pixel encoding -- 8 bits per color, organized as blue/green/red
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      # all print statements should use a rospy.log_ form, don't print!
      rospy.loginfo("Conversion failed")

    # we could do anything we want with the image here
    # for now, we'll blur using a median blur


    cv2.imshow("rgb", cv_image)

    imgHSV = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
    cv2.imshow("image_view", imgHSV)

    hsv_min2 = np.array([0,0,150])
    hsv_max2 = np.array([255,255,255])


    # White mask
    img_thr2 = cv2.inRange(imgHSV,hsv_min2,hsv_max2)
    cv2.imshow("image_thr2", img_thr2)

    # Blur before blue mask
    imgHSV = cv2.medianBlur(imgHSV,3)
    cv2.imshow("image_view_blur1", imgHSV)


    hsv_min = np.array([80,50,70])
    hsv_max = np.array([140,255,255])



    img_thr = cv2.inRange(imgHSV,hsv_min,hsv_max)
    cv2.imshow("image_thr", img_thr)

    

    img_thr = cv2.bitwise_or(img_thr,img_thr2)

    img_thr = cv2.medianBlur(img_thr,7)
    cv2.imshow("image_thr_blur", img_thr)

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
     
    params.filterByColor = True
    params.blobColor = 255
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 80
    params.maxArea = 1000
    
    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = .6          # sqaure is 0.785
    params.maxCircularity = 0.95
     
    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.95
     
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.2
     
    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
    else :
        detector = cv2.SimpleBlobDetector_create(params)

    keypoints = detector.detect(img_thr)

    if keypoints:
        x = keypoints[0].pt[0]
        y = keypoints[0].pt[1]
        t = rospy.get_time()
        self.pub.publish(x = x, y = y, t = t)

        im_with_keypoints = cv2.drawKeypoints(img_thr, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.imshow("Keypoints", im_with_keypoints)
    
    #show the image
    #cv2.imshow("image_view", imgHSV)
    #cv2.imshow("image_view2", cv_image)
    cv2.waitKey(3)
    

  def callback_field(self,data):
    """ This is a callback which recieves images and processes them. """
    # convert image into openCV format
    bridge = CvBridge()
    try:
      # bgr8 is the pixel encoding -- 8 bits per color, organized as blue/green/red
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      # all print statements should use a rospy.log_ form, don't print!
      rospy.loginfo("Conversion failed")

    # we could do anything we want with the image here
    # for now, we'll blur using a median blur


    cv2.imshow("rgb", cv_image)

    imgHSV = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
    cv2.imshow("image_view", imgHSV)

    imgHSV = cv2.medianBlur(imgHSV,3)
    cv2.imshow("image_view_blur1", imgHSV)

    gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    gray = cv2.bilateralFilter(gray, 11, 17, 17)
    edged = cv2.Canny(gray,30,200)
    (cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
    screenCnt = None

    for c in cnts:
    # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
     
    # if our approximated contour has four points, then
    # we can assume that we have found our screen
        if len(approx) == 4:
            screenCnt = approx
            break

    cv2.drawContours(cv_image, [screenCnt], -1, (0, 255, 0), 3)
    cv2.imshow("Contours", cv_image)
    #hsv_min = np.array([0,0,100])
    #hsv_max = np.array([255,255,255])


    #img_thr = cv2.inRange(imgHSV,hsv_min,hsv_max)
    #cv2.imshow("image_thr", img_thr)

    #img_thr = cv2.medianBlur(img_thr,5)
    #cv2.imshow("image_thr_blur", img_thr)

    #ret,thresh = cv2.threshold(img_thr,127,255,0)
    #contours, hierarchy = cv2.findContours(img_thr,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    #cv2.imshow("grey",thresh)
    #cv2.drawContours(img_thr, contours, -1, (0,255,0), 3)
    #cv2.imshow("contours",img_thr)

    '''# Find index of largest contour
    areas = [cv2.contourArea(c) for c in contours]
    max_index = np.argmax(areas)
    cnt = contours[max_index]

    x,y,w,h = cv2.boundingRect(cnt)
    cv2.rectangle(img_thr,(x,y),(x+w,y+h),(0,255,0),2)
    cv2.imshow("rect",img_thr)'''

    '''
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
     
    params.filterByColor = True
    params.blobColor = 255
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 100
    params.maxArea = 400
    
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = .7          # sqaure is 0.785
    params.maxCircularity = 0.9
     
    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.87
     
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.5
     
    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
    else :
        detector = cv2.SimpleBlobDetector_create(params)

    keypoints = detector.detect(img_thr)

    if keypoints:
        x = keypoints[0].pt[0]
        y = keypoints[0].pt[1]
        t = rospy.get_time()
        self.pub.publish(x = x, y = y, t = t)

        im_with_keypoints = cv2.drawKeypoints(img_thr, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.imshow("Keypoints", im_with_keypoints)
    '''

    #show the image
    #cv2.imshow("image_view", imgHSV)
    #cv2.imshow("image_view2", cv_image)
    cv2.waitKey(3)

if __name__ == '__main__':
    image_blur()
    try:  
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()
