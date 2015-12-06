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

class image_blur:

  def __init__(self):
    # initialize a node called imaging
    rospy.init_node("imaging")

    # create a window to display results in
    cv2.namedWindow("image_view", 1)

    # subscribe to proper topic
    self.image_sub = rospy.Subscriber("camera/rgb/image_color", Image, self.callback)

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

    hsv_min = np.array([100,0,70])
    hsv_max = np.array([255,255,255])

    img_thr = cv2.inRange(imgHSV,hsv_min,hsv_max)
    cv2.imshow("image_thr", img_thr)

    img_thr = cv2.medianBlur(img_thr,5)
    cv2.imshow("image_thr_blur", img_thr)
    
    
    circles = cv2.HoughCircles(img_thr, cv.CV_HOUGH_GRADIENT, 1, 10)
    circles = np.round(circles[0,:]).astype("int")
    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(img_thr,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(img_thr,(i[0],i[1]),2,(0,0,255),3)

    cv2.imshow('detected circles',img_thr)
    
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
	

if __name__ == '__main__':
    image_blur()
    try:  
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()
