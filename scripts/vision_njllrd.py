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

M_global = None
maxWidth_global = None
maxHeight_global = None
calibrated = 0
block_array = None

class image_blur:

  def __init__(self):
    # initialize a node called imaging
    rospy.init_node("vision_njllrd")

    # Create publisher
    self.pub = rospy.Publisher('ball_position',ball, queue_size=10)
    self.pub_block = rospy.Publisher('block_position',block, queue_size=10)
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
    if calibrated == 0:
        self.track_blocks(cv_image)
        if block_array is not None:
            self.calibrate_field(cv_image)
    else:
        # Warp image
        warp = cv2.warpPerspective(cv_image, M_global, (maxWidth_global+350, maxHeight_global))
        cv2.imshow("warped",warp)
        cv2.waitKey(3)      
        self.track_ball(warp)
        self.track_blocks(warp)

  def calibrate_field(self,cv_image):
    global block_array
    global calibrated
    global M_global
    global maxHeight_global
    global maxWidth_global
    #print block_array
    #print block_array.size
    screenCnt = np.array([[block_array[-8],block_array[-7]],[block_array[-6],block_array[-5]],[block_array[-4],block_array[-3]],[block_array[-2],block_array[-1]]])
    if screenCnt is not None:
        #pts = screenCnt.reshape(4, 2)
        pts = screenCnt
        print pts
        rect = np.zeros((4, 2), dtype = "float32")

        # the top-left point has the smallest sum whereas the
        # bottom-right has the largest sum
        s = pts.sum(axis = 1)
        rect[0] = pts[np.argmin(s)]
        pts = np.delete(pts,np.argmin(s),axis=0)
        s = pts.sum(axis = 1)
        rect[2] = pts[np.argmax(s)]
        pts = np.delete(pts,np.argmax(s),axis=0)

        # compute the difference between the points -- the top-right
        # will have the minumum difference and the bottom-left will
        # have the maximum difference
        diff = np.diff(pts, axis = 1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]

        # Check differences in x and y
        store_now = 0 
        diff1 = abs(rect[0,1] - rect[1,1])
        diff2 = abs(rect[1,0] - rect[2,0])
        diff3 = abs(rect[2,1] - rect[3,1])

        #print "before"
        #print rect


        '''#extend area to include blocks
        field_extend_x = 50
        field_extend_y = 50
        rect[0,0] -= field_extend_x
        rect[3,0] -= field_extend_x
        rect[0:1,1] -= field_extend_y
        rect[2:3,1] += field_extend_y
        #print "after"
        #print rect'''

        # now that we have our rectangle of points, let's compute
        # the width of our new image
        (tl, tr, br, bl) = rect
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))

        # ...and now for the height of our new image
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))

        # take the maximum of the width and height values to reach
        # our final dimensions
        maxWidth = max(int(widthA), int(widthB))
        maxHeight = max(int(heightA), int(heightB))

        # construct our destination points which will be used to
        # map the screen to a top-down, "birds eye" view
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype = "float32")

        # calculate the perspective transform matrix and warp
        # the perspective to grab the screen
        M = cv2.getPerspectiveTransform(rect, dst)
        warp = cv2.warpPerspective(cv_image, M, (maxWidth, maxHeight))

        if diff1 < 50 and diff2 < 50 and diff3 < 50:
            store_now = 1

        if store_now == 1:
            M_global = M
            maxWidth_global = maxWidth
            maxHeight_global = maxHeight
            calibrated = 1
            print calibrated

    cv2.imshow("warped",warp)

    height, width, channels = warp.shape

    rospy.set_param('/image_height', height)
    rospy.set_param('/image_width', width)
    #show the image
    #cv2.imshow("image_view", imgHSV)
    #cv2.imshow("image_view2", cv_image)
    cv2.waitKey(3)

  def track_ball(self,cv_image):
    # Image is a np array (not a ros msg)
    imgHSV = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
    cv2.imshow("image_view", imgHSV)

    imgHSV = cv2.medianBlur(imgHSV,3)
    cv2.imshow("image_view_blur1", imgHSV)

    #hsv_min = np.array([150,100,70])
    #hsv_max = np.array([255,255,255])
    hsv_min = np.array([150,100,70])
    hsv_max = np.array([255,255,255])

    img_thr = cv2.inRange(imgHSV,hsv_min,hsv_max)
    cv2.imshow("image_thr", img_thr)

    img_thr = cv2.medianBlur(img_thr,5)
    cv2.imshow("image_thr_blur", img_thr)

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
     
    params.filterByColor = True
    params.blobColor = 255
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 100
    params.maxArea = 300
     
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
    cv2.waitKey(3)

  def track_blocks(self,cv_image):
    global block_array
    imgHSV = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

    if calibrated == 0: # only look at half of the image
        if rospy.get_param('/arm') == 'left':
            imgHSV = imgHSV[0:-1,0:np.size(imgHSV,1)/2]
        else:
            imgHSV = imgHSV[0:-1,np.size(imgHSV,1)/2:-1]
        cv2.imshow('cropped',imgHSV)
    # White mask
    hsv_min2 = np.array([100,0,150])
    hsv_max2 = np.array([255,255,255])

    # Threshold with white mask
    img_thr2 = cv2.inRange(imgHSV,hsv_min2,hsv_max2)
    #cv2.imshow("white mask for block detection", img_thr2)

    # Blur before blue mask
    imgHSV = cv2.medianBlur(imgHSV,3)
    #cv2.imshow("blurred image", imgHSV)

    # Blue Mask
    hsv_min = np.array([80,50,70])
    hsv_max = np.array([140,255,255])

    # Threshold with blue mask
    img_thr = cv2.inRange(imgHSV,hsv_min,hsv_max)
    #cv2.imshow("blue mask for block detection", img_thr)

    # Combine thresholds
    img_thr = cv2.bitwise_or(img_thr,img_thr2)

    # Blur combined image
    img_thr = cv2.medianBlur(img_thr,7)
    #cv2.imshow("final blurs", img_thr)

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
     
    params.filterByColor = True
    params.blobColor = 255
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 200
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
        cv2.imshow("Keypoints blocks", im_with_keypoints)

    block_array = np.array([])
    for block_pos in keypoints:
        block_array = np.append(block_array, block_pos.pt[0])
        block_array = np.append(block_array, block_pos.pt[1])
    self.pub_block.publish(block = block_array)
    cv2.waitKey(3)

if __name__ == '__main__':
    image_blur()
    try:  
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()
