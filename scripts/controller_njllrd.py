#!/usr/bin/env python
import rospy
import time
import math
import numpy
import ast
from njllrd_proj3.srv import *
from njllrd_proj3.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from game_server.msg import *
from game_server.srv import *
import baxter_interface
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import cv2
import cv2.cv as cv
from cv_bridge import CvBridge, CvBridgeError

flag = False
x_last = 0           # last stored x_position of the ball
y_last = 0           # last stored y_position of the ball
t_last = 0           # timestamp of last stored position of the ball
arm = None           # Indicator of if we are controlling right or left arm
vx = 0               # x velocity of the ball, in field frame
vy = 0               # y velocity of the ball, in field frame
ball_pos = None
field_width = .78    # m, short-side of field (only includes green area, exclude 2x4)
field_length = 1.4
x_goal1 = None      # end of goal closeste to baxter
x_goal2 = None      # end of goal farthest from baxter
field_divisions = None
field_width_pixels = None
field_length_pixels = None
block_positions = None
home_position = None
lastmode = 0
angles1 = None
angles2 = None

def controller_njllrd():
    global arm
    global angles1
    global angles2
    rospy.init_node('controller_njllrd')

    
    '''rospy.wait_for_service('/game_server/init')
    gamesvc = rospy.ServiceProxy('/game_server/init', Init)
    path = rospy.get_param("/path")
    image = cv2.imread(path)
    imagemsg = CvBridge().cv2_to_imgmsg(image, encoding="bgr8")
    receiveddata = gamesvc("baxstreetboys", imagemsg)
    receivedarm = receiveddata.arm
    rospy.set_param("/arm_njllrd", receivedarm)'''


    #DELETE THIS
    rospy.set_param('/arm_njllrd', "left")




    rospy.wait_for_service('request_endpoint')
    rospy.wait_for_service('request_orientation')

    #rospy.Subscriber('/game_server/game_state', GameState, handle_game_state)
    rospy.Subscriber('user_input', String, handle_user_input)
    # subscribe to ball position topic
    rospy.Subscriber("ball_position", ball, get_ball_velocity)

    # subscribe to block position topic
    rospy.Subscriber("block_position", block, update_block_positions)


    # Determine which arm we are controlling
    if rospy.get_param('/arm_njllrd') == "left":
        arm = "left"
    else:
        arm = "right"

    print arm
    home_time = rospy.get_time()

    limb = baxter_interface.Limb(arm)

    ##### Testing loop for vision stuff #######
    #while True:
    #    i = True 
    ###########################################
    while True:
        #print rospy.get_param('/mode')
        #print rospy.get_param('/arm')
        #print "f"
        if rospy.get_param('/mode_njllrd') == "connect_points":
            point1, point2 = get_connect_points()
            
            points = waypoints()
            
            pointi = point()
            
            pointi.x = point1.endpoint.x
            pointi.y = point1.endpoint.y
            pointi.z = point1.endpoint.z

            

            pointj = point()
            pointj.x = point2.endpoint.x
            pointj.y = point2.endpoint.y
            pointj.z = point2.endpoint.z

            points.points.append(pointj)
            points.points.append(pointi)
            
            print points.points[0]
            print points.points[1]
            print points.points
            request = rospy.ServiceProxy('connect_waypoints', connect_waypoints)
            output = request(points)
        elif rospy.get_param('/mode_njllrd') == "sweep":
            r_t = rospy.ServiceProxy('request_translate', translate)
            current_pos = request_position(yes)
            translate_success = r_t(current_pos.x + .10,current_pos.y,current_pos.z)
            home_success = go_home()
            temp_block_positions = block_positions
            needs_sweep = True
            while needs_sweep and rospy.get_param('/mode_njllrd') == "sweep":
                needs_sweep = False
                for i in range(0,len(temp_block_positions),2):
                    print "i"
                    print i
                    print temp_block_positions
                    x = temp_block_positions[i]
                    y = temp_block_positions[i+1] 
                    if arm == "right":
                        if y > field_length - .25:
                            needs_sweep = True
                    else:
                        if y < .25:
                            needs_sweep = True
                if needs_sweep:
                    sweep(origin, temp_block_positions)
            rospy.set_param('/mode_njllrd','wait')

                        
                       
        
        elif rospy.get_param('/mode_njllrd') == "wait":
            a = True
            #print "waiting"
        elif rospy.get_param('/mode_njllrd') == "field":
            global field_length_pixels
            global field_width_pixels
            field_length_pixels = rospy.get_param('/image_width')
            field_width_pixels = rospy.get_param('/image_height')
            print "here"
            print rospy.get_param("/image_height")

            print rospy.get_param("/arm_njllrd")
            # Calibrate the home position
            home_cal_succ = calibrate_home()

            # initialize the field, create plane, rotations, etc.
            origin = initialize_field()

            # get strike positions
            #angles1 = get_angles()
            #angles2 = get_angles()

            print "origin"
            print origin

            x = origin[0]
            y = origin[1]
            z = origin[2]


            r_t = rospy.ServiceProxy('request_translate', translate)
            r_r = rospy.ServiceProxy('request_rotate', rotate)

            
            home_success = go_home()        # move to home position and set the mode to defense
            rospy.set_param('/mode_njllrd','defense')
            #rospy.set_param('/mode','wait')
            #rospy.set_param('/mode','wait')
            
        elif rospy.get_param('/mode_njllrd') == "defense":
            global x_goal1
            global x_goal2
            global field_divisions
            stall_time = rospy.get_time()-home_time
            # EVERY SO OFTEN, GO HOME TO FIX ORIENTATION
            if stall_time > 20:
                home_success = go_home()
                home_time = rospy.get_time()

            # try to find the ball
            # check ball side/position
            #print ball_pos
            if ball_pos is not None:
                ball_side = get_ball_side(ball_pos,field_divisions)      # ball_side == 1 if on our side
                # calculate velocity, done in callback from ball_positions topic, stored as vx,vy
                

                #print "velocities"
                #print vx
                #print vy
                #print ball_side
                
                if ball_side == 1:
                    #print "ball on our side"
                    # check velocity
                    #print vx
                    #print vy
                    if abs(vx) < 0.03  and abs(vy) < 0.03:
                        # ball is still on our side
                        # switch to offense
                        #print "ball still"
                        rospy.sleep(1)
                        rospy.set_param('/mode_njllrd','offense')
                        print rospy.get_param('/mode_njllrd')
                        #elif (vx != 0) and (vy != 0): # ball is on our side still moving 
                    else:
                        #print "ball moving on our side"
                        # compute ball trajectory
                        x_impact = get_ball_trajectory()
                        #x_impact = ball_pos[0]
                        #print "ximpact"
                        #print x_impact
                        # if necessary, compute point to move to to block ball
                        if x_impact > x_goal1 and x_impact < x_goal2:
                            # move arm to block accordingly
                            # convert x_impact to baxter coordinates 
                            
                            #new_point = origin + numpy.dot(R,numpy.array([x_impact,0,0]))
                            new_point = origin + numpy.array([x_impact,-.05,0])
                            #print "moving to block ball"
                            translate_success = r_t(new_point[0], new_point[1], new_point[2])
                elif (abs(vx) > .05) and (abs(vy) > 0.05):
                    x_impact = get_ball_trajectory()
                    #x_impact = ball_pos[0]
                    #print "ximpact"
                    #print x_impact
                    # if necessary, compute point to move to to block ball
                    if x_impact > x_goal1 and x_impact < x_goal2:
                        # move arm to block accordingly
                        # convert x_impact to baxter coordinates 
                        
                        #new_point = origin + numpy.dot(R,numpy.array([x_impact,0,0]))
                        new_point = origin + numpy.array([x_impact,-.05,0])
                        #print "moving to block ball"
                        translate_success = r_t(new_point[0], new_point[1], new_point[2])
                   
            
        elif rospy.get_param('/mode_njllrd') == 'offense':
            # get current ball pos
            # updated automatically in topic callback, stored as ball_pos [x,y,t]
            ball_placed = True
            # check whether ball is approximately within region of dot
            if arm == 'right':
                # ball should be at x = .36, y = field_length-.22
                if ball_pos[0] > .32 and ball_pos[0] < .40 and ball_pos[1] > field_length-.22-.04 and ball_pos[1] < field_length-.22+.04:
                    ball_placed = True
                else:
                    ball_placed = False

            else:
                # ball should be at x = .36, y = .22
                if ball_pos[0] > .32 and ball_pos[0] < .40 and ball_pos[1] > .18 and ball_pos[1] < .26:
                    ball_placed = True
                else:
                    ball_placed = False

            temp_ball_pos = ball_pos
            print "Ballpos"
            print temp_ball_pos
            if ball_placed:
                # compute desired trajectory
                # to start, assume ball is at circle.
                # check straight shot, and two bank shots aiming for sides of goal
                # check to make sure no blocks in path of the ball's trajectory
                home_success = go_home()
                if arm == 'right':
                    target_point = numpy.array([field_width/2,0])
                else:
                    target_point = numpy.array([field_width/2,field_length])

                bank_offset = numpy.array([0.05,0])
                intersection = check_for_blocks(target_point,ball_pos)
                print "intersection_straight"
                print intersection 

                if intersection !=0:

                    (theta,new_target_point) = get_bank(target_point+bank_offset)
                    intersection1 = check_for_blocks(new_target_point,ball_pos)
                    intersection2 = check_for_blocks(target_point,new_target_point)
                    print "intersection_up"
                    print intersection1
                    print intersection2

                    if intersection1 != 0  or intersection2 !=0:
                        (theta,new_target_point) = get_bank(target_point-bank_offset)
                        intersection3 = check_for_blocks(new_target_point,ball_pos)
                        intersection4 = check_for_blocks(target_point,new_target_point)
                        print "intersection_down"
                        print intersection3
                        print intersection4

                # move to starting point
                # home_success = go_home()
                rospy.set_param("/striking", "True")
                if intersection == 0:
                    # shoot straight
                    if arm == 'right':
                        strike_end_point = numpy.array([origin[0]+ball_pos[0],origin[1]-(ball_pos[1]-.02), origin[2]])
                        translate_success = r_t(strike_end_point[0],strike_end_point[1],strike_end_point[2])
                        #limb.set_joint_position_speed(.2)
                        #limb.set_joint_positions(angles1)
                        #limb.set_joint_position_speed(1)
                        #limb.set_point_positions(angles2)

                    else:
                        strike_end_point = numpy.array([origin[0] + ball_pos[0],origin[1]-(ball_pos[1]+.02), origin[2]])
                        rotate_success = r_r(math.pi/8)
                        translate_success = r_t(strike_end_point[0],strike_end_point[1],strike_end_point[2])
                        #limb.set_joint_position_speed(.2)
                        #limb.set_joint_positions(angles1)
                        #limb.set_joint_position_speed(1)
                        #limb.set_point_positions(angles2)
                    
                elif intersection1 == 0 and intersection2 == 0:
                    # shoot up
 
                    if arm == 'right':
                        rospy.set_param("/striking", "False")
                        strike_start_point = numpy.array([temp_ball_pos[0]-.2*math.sin(theta), temp_ball_pos[1]+.2*math.cos(theta), 0])
                        print "strike_start_point"
                        print strike_start_point
                        print numpy.array([strike_start_point[0]+origin[0],origin[1]-strike_start_point[1], strike_start_point[2]+origin[2]])
                        print "origin"
                        print origin
                        translate_success = r_t(strike_start_point[0]+origin[0],origin[1]-strike_start_point[1], strike_start_point[2]+origin[2])


                        rospy.sleep(2)

                        rotate_success = r_r(theta)

                        rospy.sleep(2)

                        rospy.set_param("/striking", "banked")
                        strike_end_point = numpy.array([temp_ball_pos[0]+.02*math.sin(theta), temp_ball_pos[1]-.02*math.cos(theta), 0])
                        print "strike_end_point"
                        print strike_end_point
                        print numpy.array([strike_end_point[0]+origin[0],origin[1]-strike_end_point[1], strike_end_point[2]+origin[2]])
                        translate_success = r_t(strike_end_point[0]+origin[0],-strike_end_point[1]+origin[1], strike_end_point[2]+origin[2])
                        print "origin"
                        print origin
                        rospy.set_param("/striking", "False")
                    else:
                        rospy.set_param("/striking", "False")
                        strike_start_point = numpy.array([temp_ball_pos[0]-.2*math.sin(theta), temp_ball_pos[1]-.2*math.cos(theta), 0])
                        print "strike_start_point"
                        print strike_start_point
                        print numpy.array([strike_start_point[0]+origin[0],origin[1]-strike_start_point[1], strike_start_point[2]+origin[2]])
                        print "origin"
                        print origin
                        translate_success = r_t(strike_start_point[0]+origin[0],origin[1]-strike_start_point[1], strike_start_point[2]+origin[2])


                        rospy.sleep(2)

                        rotate_success = r_r(-theta)

                        rospy.sleep(2)

                        rospy.set_param("/striking", "banked")
                        strike_end_point = numpy.array([temp_ball_pos[0]+.02*math.sin(theta), temp_ball_pos[1]+.02*math.cos(theta), 0])
                        print "strike_end_point"
                        print strike_end_point
                        print numpy.array([strike_end_point[0]+origin[0],origin[1]-strike_end_point[1], strike_end_point[2]+origin[2]])
                        translate_success = r_t(strike_end_point[0]+origin[0],-strike_end_point[1]+origin[1], strike_end_point[2]+origin[2])
                        print "origin"
                        print origin
                        rospy.set_param("/striking", "False")

                elif intersection3 == 0 and intersection4 == 0:
                    # shoot down

                    if arm == 'right':
                        #strike_start_point = 
                        #translate_success = 

                        rotate_success = r_r(theta)

                        #strike_end_point = 
                        #translate_success = 
                    else:
                        #strike_start_point = 
                        #translate_success = 
                        print "strike_start_point"
                        print strike_start_point
                        print theta

                        rotate_success = r_r(-theta)

                        #strike_end_point = 
                        #translate_success = 
                        print "strike_end_point"
                        print strike_end_point
                    
                else:
                    # blast through the middle
                    if arm == 'right':
                        strike_end_point = numpy.array([origin[0]+ball_pos[0],origin[1]-(ball_pos[1]-.02), origin[2]])
                        translate_success = r_t(strike_end_point[0],strike_end_point[1],strike_end_point[2])
                        #limb.set_joint_position_speed(.2)
                        #limb.set_joint_positions(angles1)
                        #limb.set_joint_position_speed(1)
                        #limb.set_point_positions(angles2)
                    else:
                        strike_end_point = numpy.array([origin[0]+ball_pos[0],origin[1]-(ball_pos[1]+.02), origin[2]])
                        rotate_success = r_r(math.pi/8)
                        translate_success = r_t(strike_end_point[0],strike_end_point[1],strike_end_point[2])
                        #limb.set_joint_position_speed(.2)
                        #limb.set_joint_positions(angles1)
                        #limb.set_joint_position_speed(1)
                        #limb.set_point_positions(angles2)

                # end strike ball
                rospy.set_param("/striking", "False")
                #move to home, set to defense
                home_success = go_home()
                print("HERE")
                rospy.set_param('/mode_njllrd', "defense")

                '''
                target_point = numpy.array([field_width/2,0,0])
                print "target_point"
                print target_point
                (theta,target_point) = get_bank(target_point)
                print "new_target_point"
                print target_point
                print "theta"
                print theta
                rotate_success = r_r(theta)
                '''
            else:
                home_success = go_home()
                rospy.set_param('/mode_njllrd', "defense")


        # time.sleep(10)
    rospy.spin()

def make_point(data):
    the_point = data
    the_real_point = point()
    the_real_point.x = the_point.endpoint.x
    the_real_point.y = the_point.endpoint.y
    the_real_point.z = the_point.endpoint.z
    return the_real_point

def make_point_from_array(data):
    the_point = data
    the_real_point = point()
    the_real_point.x = the_point[0]
    the_real_point.y = the_point[1]
    the_real_point.z = the_point[2]
    return the_real_point

def handle_user_input(data):
    global flag
    flag = True
    #print "got here"


def request_position():
    yes = True
    request = rospy.ServiceProxy('request_endpoint', request_endpoint)
    output = request(yes)
    return output

def request_config():
    yes = True
    request = rospy.ServiceProxy('request_orientation', request_orientation)
    output = request(yes)
    return output

def get_plane_points():
    print "Requesting three plane points. Enter point 1"
    global flag
    flag = False

    #wait for user to hit enter
    while flag == False:
        x = 1
        #loop
    point1 = request_position()
    flag = False

    print "Enter point 2"
    while flag == False:
        x = 1
        #loop

    #wait for user to hit enter
    point2 = request_position()
    flag = False

    print "Enter point 3"
    while flag == False:
        x = 1
        #loop

    #wait for user to hit enter
    point3 = request_position()
    flag = False
    #publish waypoints to robot_interface
    #print (point1, point2, point3)
    print "plane created"
    return (point1, point2, point3)
    rect
def make_rotation_matrix(plane_normal):
    z = numpy.array([0, 0, 1])
    k = numpy.cross(z,plane_normal)
    kx = k[0]
    ky = k[1]
    kz = k[2]

    theta = math.acos(numpy.dot(z,plane_normal))
    v = 1-math.cos(theta)
    c = math.cos(theta)
    s = math.sin(theta)

    R = numpy.array([[kx**2*v+c, kx*ky*v - kz*s, kx*kz*v + ky*s],[kx*kz*v + kz*s, ky**2*v + c, ky*kz*v - kx*s],[kx*kz*v - ky*s, ky*kz*v + kx*s, kz**2 * v +c]])
    return R

def get_ball_velocity(data):
    # This is a callback function for the ball_positions topic, everytime a new postion is published, the velocity is computed in this function
    # Right now, the position is in the image frame, need to change this to field or inertial frame

    global field_width_pixels
    if field_width_pixels: 

        global ball_pos
        global x_last
        global y_last
        global t_last
        global vx
        global vy
        
        global field_width

        image_x = data.x
        image_y = data.y
        t = data.t
        '''print "field width"
        print field_width_pixels
        print "field length"
        print field_length_pixels
        print "image_x"
        print image_x
        print "image_y"
        print image_y'''
        # transform position and scale from image to field
        ######## HERE IS THE ISSUE.. NEED TO TRANSFORM IMAGE TO FIELD!!!!!! ############

        # x is measured from bottom of field, top of image. Also needs to be scaled.
        x = (field_width_pixels - image_y)*field_width/field_width_pixels

        # y needs to be scaled, and direction needs to be reversed
        y = image_x*field_length/field_length_pixels

        #print "field x"
        #print x
        #print "field y"
        #print y
        
        # store ball position as an array
        ball_pos = numpy.array([x,y,t])

        # compute velocity
        dx = x-x_last
        dy = y-y_last
        dt = t-t_last
        vx = dx/dt
        vy = dy/dt

        x_last = x;
        y_last = y;
        t_last = t;

def initialize_field():
    global field_length
    global field_width
    global x_goal1
    global x_goal2
    global field_divisions
    global flag
    A1_length = .20      # m, length of A section closest to goal
    A2_length = .10      # m, length of A section closest to center
    B_length = .25       # m, length of B section
    field_width = .78    # m, short-side of field (only includes green area, exclude 2x4)
    field_length = 1.4

    x_goal1 = 0.21         # field coordinates, end of goal closest to baxter
    x_goal2 = 0.51         # field coordinates, end of goal farthest from baxter

    #half_field = A1_length + B_length + A2_length
    
    half_field = field_length/2
    # Create an array of positions relative to 0 (bottom left corner) that divide the field into sections
    field_divisions = numpy.array([0, A1_length, A1_length+B_length, half_field, half_field+A2_length, half_field + A2_length + B_length, field_length])

    # Make transformation from field/image to robot frame
    #point1, point2, point3 = get_plane_points()

    while flag == False:
        x = 1
        #loop
    point1 = request_position()
    flag = False

    pointc = numpy.array([point1.endpoint.x, point1.endpoint.y, point1.endpoint.z])
    #pointb = numpy.array([point2.endpoint.x, point2.endpoint.y, point2.endpoint.z])

    # Move pointc to the bottom left corner of the field. This is kind of an approximation since the table may not be planar. Should be good enough
    if arm == "right":  # need to add some offset to bottom left corner
        pointc = numpy.array([point1.endpoint.x, point1.endpoint.y + field_length, point1.endpoint.z])

    #plane_vec = numpy.cross(pointa-pointb, pointb-pointc)
    #plane_normal = plane_vec/numpy.linalg.norm(plane_vec)
    #R = make_rotation_matrix(plane_normal)
    return pointc

def calibrate_home():
    global flag
    global home_position
    flag = False
    r_h_cal = rospy.ServiceProxy('request_home_calibrate', home_calibrate)      # calibrates the home position at the start

    print "Calibrate the home position"
    #wait for user to hit enter
    while flag == False:
        x = 1
        #loop
    # h_cal_success, home_position = r_h_cal(True)
    print rospy.get_param('/arm_njllrd')
    data = r_h_cal(True)
    h_cal_success = data.success
    home_position = data.home_position
    flag = False
    return h_cal_success

def go_home():
    r_t = rospy.ServiceProxy('request_translate', translate)
    translate_success = r_t(home_position[0],home_position[1],home_position[2])


    r_h = rospy.ServiceProxy('request_home', home)      # go to a "home" position, starting defensive position in front of goal
    home_success = r_h(True)
    #rospy.set_param('/mode','defense')

    return home_success

def get_ball_side(ball_position,field_divisions):
    # Data = the ball pos array
    y = ball_position[1]  # this is in image coordinates, field divisions is in field coordinates... need a transform!!
    if arm == "left":
        my_side = numpy.array([field_divisions[0],field_divisions[3]]) 
    elif arm == "right":
        my_side = numpy.array([field_divisions[3],field_divisions[6]])

    # Check if ball is on our side of the field
    if (y > my_side[0]) and (y <= my_side[1]):
        ball_onside = 1         # ball is on our side
        # We can switch to offense now... maybe want to add a velocity checker to make sure the ball is not moving too quickly, or we might want to wait until the ball is essentially still
    else:
        ball_onside = 0         # ball is not on our side

    return ball_onside

def get_ball_trajectory():
    # calculate the impact location of the ball using linear trajectory ie. y = mx+b  find x intercept
    global field_length
    global vx
    global vy
    global ball_pos
    x = ball_pos[0]
    y = ball_pos[1]
    if arm == "left":
        y_offset = 0
        if vy < 0:
            m = vy/vx
            b = y-m*x

            x_impact = (y_offset - b)/m
        else:
            x_impact = None
    else:
        y_offest = -field_length
        if vy > 0:
            m = vy/vx
            b = y-m*x

            x_impact = (y_offset - b)/m
        else: 
            x_impact = None

    return x_impact

def check_for_blocks(target_point,start_point):
    global block_positions
    intersection = 0
    temp_block_positions = block_positions
    if temp_block_positions.any:
        point1 = [start_point[0], start_point[1]]
        point2 = [target_point[0],target_point[1]]
        line1 = [point1,point2]
        for i in range(0,len(temp_block_positions),2):
            block_x = temp_block_positions[i]
            block_y = temp_block_positions[i+1]
            point1b = [block_x+0.045, block_y]
            point2b = [block_x-0.045, block_y]
            line2 = [point1b,point2b]
            # check intersection
            #print "line1"
            #print line1
            #print "line2"
            #print line2 
            check = line_intersection(line1,line2)
            #print "check"
            #print check
            if check == 1:
                intersection = 1
    return intersection


def update_block_positions(data):
    # Callback for block position topic which updates the global variable block_positions everytime a new block_position msg is published
    global block_positions
    global field_length
    global field_width
    global field_width_pixels
    global field_length_pixels

    block_positions = numpy.array(data.block)
    if block_positions.any:
        for i in range(0,len(block_positions),2):

            image_x = block_positions[i]
            image_y = block_positions[i+1]
            '''print "image block points"
            print image_x
            print image_y'''


            x = (field_width_pixels - image_y)*field_width/field_width_pixels
            y = image_x*field_length/field_length_pixels
            block_positions[i] = x
            block_positions[i+1] = y




def line_intersection(line1, line2):
    m = (line1[1][0]-line1[0][0]) / (line1[1][1]-line1[0][1]) 
    b = line1[0][0]-m*line1[0][1]
    '''print "m"
    print m
    print "b"
    print b'''
    check = m*line2[0][1] + b
    if check < line2[0][0] and check > line2[1][0]:
        intersection = 1
    else:
        intersection = 0
    '''
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        intersection = 0

    else:
        intersection = 1
    '''
    
    return intersection

def sweep(origin, temp_block_positions):
    r_t = rospy.ServiceProxy('request_translate', translate)
    
    if temp_block_positions.any:
        for i in range(0,len(temp_block_positions),2):
            x = temp_block_positions[i]
            y = temp_block_positions[i+1] 
            print "origin_sweep"
            print origin
            if arm == "right":
                if y > field_length - .25:
                    # move to y position of origin
                    #sweep_start_point = numpy.array([origin[0], origin[1],origin[2]])
                    #translate_success = r_t(sweep_start_point[0],sweep_start_point[1],sweep_start_point[2])

                    sweep_start_point = numpy.array([x+origin[0], origin[1]-0.02 ,origin[2]])
                    translate_success = r_t(sweep_start_point[0],sweep_start_point[1],sweep_start_point[2])
                    
                    #sweep from right to left at x = x
                    sweep_end_point = numpy.array([x+origin[0],origin[1]+field_length-.30, origin[2]])
                    translate_success = r_t(sweep_end_point[0],sweep_end_point[1],sweep_end_point[2])

            else:
                if y < .25:
                    print "blockx"
                    print x
                    print "blocky"
                    print y
                   # move to y position of origin
                    #sweep_start_point = numpy.array([origin[0], origin[1],origin[2]])
                    #translate_success = r_t(sweep_start_point[0],sweep_start_point[1],sweep_start_point[2])

                    sweep_start_point = numpy.array([x+origin[0], origin[1]-0.02,origin[2]])
                    translate_success = r_t(sweep_start_point[0],sweep_start_point[1],sweep_start_point[2])
                    
                    #sweep from right to left at x = x
                    sweep_end_point = numpy.array([x+origin[0],-.30+origin[1], origin[2]])
                    translate_success = r_t(sweep_end_point[0],sweep_end_point[1],sweep_end_point[2])

def handle_game_state(data):
    global lastmode
    mode = data.current_phase
    print "mode"
    print mode

    if abs(mode-lastmode) > 0:
        if mode == 3:
            rospy.set_param('/mode_njllrd', "defense")
            lastmode = 3
        elif mode == 2:
            rospy.set_param('/mode_njllrd', "sweep")
            lastmode = 2
        elif mode == 1:
            rospy.set_param('/mode_njllrd', "field")
            lastmode = 1



def get_bank(target_point):
    global field_width
    global field_length
    global ball_pos

    if target_point[0] >= field_width/2:
        c = field_width-target_point[0]
        d = field_width-ball_pos[0]
    else:
        c = target_point[0]
        d = ball_pos[0]
    l = abs(target_point[1]-ball_pos[1])
    f = l/(1+d/c)
    e = l-f
    theta2 = math.atan(e/d)
    theta = math.pi/2 - theta2
    if arm == 'left':
        if target_point[0] >= field_width/2: #shooting up
            new_target_point = numpy.array([ball_pos[0]+e,field_width,0])
        else: #shooting down
            new_target_point = numpy.array([ball_pos[0]+e,0,0])
    else:
        if target_point[0] >= field_width/2: #shooting up
            new_target_point = numpy.array([e,field_width,0])
        else: #shooting down
            new_target_point = numpy.array([e,0,0])

    return (theta,new_target_point)

def get_angles():
    limb = baxter_interface.Limb(rospy.get_param('arm_njllrd'))
   
    global flag
    flag = False

    #wait for user to hit enter
    while flag == False:
        x = 1

    angles = limb.joint_angle(rospy.get_param('/arm_njllrd') + '_w0')

    return angles

if __name__ == "__main__":
    controller_njllrd()

