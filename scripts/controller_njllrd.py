#!/usr/bin/env python
import rospy
import time
import math
import numpy
import ast
from njllrd_proj3.srv import *
from njllrd_proj3.msg import *
from std_msgs.msg import String

flag = False
x_last = 0           # last stored x_position of the ball
y_last = 0           # last stored y_position of the ball
t_last = 0           # timestamp of last stored position of the ball
arm = None           # Indicator of if we are controlling right or left arm
vx = 0               # x velocity of the ball, in field frame
vy = 0               # y velocity of the ball, in field frame
ball_pos = None
field_length = None
field_width = None
x_goal1 = None      # end of goal closeste to baxter
x_goal2 = None      # end of goal farthest from baxter
field_divisions = None
field_width_pixels = None
field_length_pixels = None

def controller_njllrd():
    rospy.init_node('controller_njllrd')
    rospy.wait_for_service('request_endpoint')
    rospy.wait_for_service('request_orientation')
    rospy.Subscriber('user_input', String, handle_user_input)
    # subscribe to ball position topic
    rospy.Subscriber("ball_position", ball, get_ball_velocity)

    # Determine which arm we are controlling
    global arm
    if rospy.get_param('/arm') == "left":
        arm = "left"
    else:
        arm = "right"

    print arm

    ##### Testing loop for vision stuff #######
    #while True:
    #    i = True
    ###########################################
    while True:
        if rospy.get_param('/mode') == "connect_points":
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
        
        elif rospy.get_param('/mode') == "field":
            # Calibrate the home position
            home_cal_succ = calibrate_home()

            # initialize the field, create plane, rotations, etc.
            (R,origin) = initialize_field()

            print "origin"
            print origin

            x = origin[0]
            y = origin[1]
            z = origin[2]


            r_t = rospy.ServiceProxy('request_translate', translate)
            r_r = rospy.ServiceProxy('request_rotate', rotate)
            
            new_point = origin + numpy.dot(R,numpy.array([0, 0, -0.01]))
            print "new Points"
            print new_point

            translate_success = r_t(new_point[0], new_point[1], new_point[2])

            global field_length_pixels
            global field_width_pixels
            field_length_pixels = rospy.get_param("/image_width")
            field_width_pixels = rospy.get_param("/image_height")
            print "here"
            print rospy.get_param("/image_height")
            home_success = go_home()        # move to home position and set the mode to defense
            
        elif rospy.get_param('/mode') == "defense":
            global x_goal1
            global x_goal2
            global field_divisions
            # try to find the ball
            # check ball side/position
            ball_side = get_ball_side(ball_pos,field_divisions)      # ball_side == 1 if on our side
            # calculate velocity, done in callback from ball_positions topic, stored as vx,vy
            if ball_side == 1:
                #print "ball on our side"
                # check velocity
                print vx
                print vy
                if vx < 0.01 and vy < 0.01:
                    # ball is still on our side
                    # switch to offense
                    #print "ball still"
                    rospy.set_param('/mode','offense')
                elif (vx != 0) and (vy != 0): # ball is on our side still moving 
                    #print "ball moving on our side"
                    # compute ball trajectory
                    x_impact = get_ball_trajectory()
                    # if necessary, compute point to move to to block ball
                    if x_impact > x_goal1 and x_impact < x_goal2:
                        # move arm to block accordingly
                        # convert x_impact to baxter coordinates 
                        print "ximpact"
                        print x_impact
                        new_point = origin + numpy.dot(R,numpy.array([x_impact,0,0]))
                        #print "moving to block ball"
                        translate_success = r_t(new_point[0], new_point[1], new_point[2])
            elif (vx != 0) and (vy != 0):
                print "ball positions velocity not 0"
                print ball_pos
                translate_success = r_t(ball_pos[0], ball_pos[1], origin[2]) #move to ball TEST

                '''# ball is on opponents side
                #print "ball not on our side"
                # compute ball trajectory
                x_impact = get_ball_trajectory()
                # if necessary, compute point to move to to block ball
                if x_impact > x_goal1 and x_impact < x_goal2:
                    # move arm to block accordingly
                    # convert x_impact to baxter coordinates 
                    new_point = origin + numpy.dot(R,numpy.array([x_impact,0,0]))
                    #print "moving to block ball"
                    translate_success = r_t(new_point[0], new_point[1], new_point[2])
                    '''

            else: # move to ball TEST
                print "ball positions"
                print ball_pos
                translate_success = r_t(ball_pos[0], ball_pos[1], origin[2])

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
        print "a"
        print field_width_pixels
        print "b"
        print image_x
        print "c"
        print field_width
        # transform position and scale from image to field
        ######## HERE IS THE ISSUE.. NEED TO TRANSFORM IMAGE TO FIELD!!!!!! ############

        # x is measured from bottom of field, top of image. Also needs to be scaled.
        x = (field_width_pixels - image_x)*field_width/field_width_pixels

        # y needs to be scaled, and direction needs to be reversed
        y = -image_y*field_length/field_length_pixels

        
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
    A1_length = .20      # m, length of A section closest to goal
    A2_length = .10      # m, length of A section closest to center
    B_length = .25       # m, length of B section
    field_width = .762    # m, short-side of field (only includes green area, exclude 2x4)

    x_goal1 = 0.20         # field coordinates, end of goal closest to baxter
    x_goal2 = 0.52         # field coordinates, end of goal farthest from baxter

    #half_field = A1_length + B_length + A2_length
    field_length = 1.51
    half_field = field_length/2
    # Create an array of positions relative to 0 (bottom left corner) that divide the field into sections
    field_divisions = numpy.array([0, A1_length, A1_length+B_length, half_field, half_field+A2_length, half_field + A2_length + B_length, field_length])

    frame_width = 1.51 #meters
    frame_length = .762 #meters

    # Make transformation from field/image to robot frame
    scale_factor = 0.01
    point1, point2, point3 = get_plane_points()
    pointa = numpy.array([point1.endpoint.x, point1.endpoint.y, point1.endpoint.z])
    pointb = numpy.array([point2.endpoint.x, point2.endpoint.y, point2.endpoint.z])

    # Move pointc to the bottom left corner of the field. This is kind of an approximation since the table may not be planar. Should be good enough
    if arm == "right":  # need to add some offset to bottom left corner
        #pointc = numpy.array([point3.endpoint.x, point3.endpoint.y + field_length/2, point3.endpoint.z])
        pointc = numpy.array([point3.endpoint.x, point3.endpoint.y, point3.endpoint.z])
    else:
        pointc = numpy.array([point3.endpoint.x, point3.endpoint.y, point3.endpoint.z])

    plane_vec = numpy.cross(pointa-pointb, pointb-pointc)
    plane_normal = plane_vec/numpy.linalg.norm(plane_vec)
    R = make_rotation_matrix(plane_normal)

    return (R, pointc)

def calibrate_home():
    global flag
    flag = False
    r_h_cal = rospy.ServiceProxy('request_home_calibrate', home_calibrate)      # calibrates the home position at the start

    print "Calibrate the home position"
    #wait for user to hit enter
    while flag == False:
        x = 1
        #loop
    h_cal_success = r_h_cal(True)
    flag = False
    return h_cal_success

def go_home():
    r_h = rospy.ServiceProxy('request_home', home)      # go to a "home" position, starting defensive position in front of goal
    home_success = r_h(True)
    rospy.set_param('/mode','defense')

    return home_success

def get_ball_side(ball_position,field_divisions):
    # Data = the ball pos array
    x = ball_position[0]  # this is in image coordinates, field divisions is in field coordinates... need a transform!!
    if arm == "left":
        my_side = numpy.array([field_divisions[0],field_divisions[3]]) 
    elif arm == "right":
        my_side = numpy.array([field_divisions[3],field_divisions[6]])

    # Check if ball is on our side of the field
    if (x > my_side[0]) and (x <= my_side[1]):
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
    else:
        y_offest = -field_length

    m = vy/vx
    b = y-m*x

    x_impact = (y_offset - b)/m
    return x_impact


if __name__ == "__main__":
    controller_njllrd()

