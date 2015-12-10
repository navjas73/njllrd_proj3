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

def controller():
    rospy.init_node('controller')
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

        home_success = go_home()        # move to home position and set the mode to defense
        
    elif rospy.get_param('/mode') == "defense":
        # try to find the ball
        # check ball side/position
        ball_side = get_ball_side(ball_pos,field_divisions)      # ball_side == 1 if on our side
        # calculate velocity, done in callback from ball_positions topic, stored as vx,vy
        if ball_side == 1:
            # check velocity
            if vx < 0.1 & vy < 0.1
                # ball is still on our side
                # switch to offense
                rospy.set_param('/mode','offense')
            else: # ball is on our side still moving 
                # compute ball trajectory
                # if necessary, compute point to move to to block ball
                # move arm to block accordingly

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

def get_connect_points():
    global flag
    flag = False
    #wait for user to hit enter
    while flag == False:
        x = 1
        #loop
    if rospy.get_param('/mode') == "RRT" or rospy.get_param('/mode') == "BIRRT":
        point1 = request_config()
    else:
        point1 = request_position()
    flag = False

    while flag == False:
        x = 1
        #loop

    #wait for user to hit enter
    if rospy.get_param('/mode') == "RRT" or rospy.get_param('/mode') == "BIRRT":
        point2 = request_config()
    else:
        point2 = request_position()

    #publish waypoints to robot_interface

    return (point1, point2)

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

def draw_letter(data,scale_factor,R,pointc):
    request = rospy.ServiceProxy('connect_waypoints', connect_waypoints)
    stroke_request = waypoints()
    for stroke in data[:-1]: # Get all but the last, which is the point to end on
            new_stroke = numpy.array([])
            first_point = True
            for orig_point in stroke:
                new_point = scale_factor*orig_point
                new_point = numpy.dot(R,new_point)
                
                if first_point == True:
                    new_stroke = new_point
                    first_point = False
                else:
                    new_stroke = numpy.vstack((new_stroke, new_point))
    
            for new_stroke_point in new_stroke:
                
                new_stroke_point = new_stroke_point+pointc
                
                stroke_request.points.append(make_point_from_array(new_stroke_point))

            
            # Before we start writing, lift pen and move to first point
            go_to_stroke = move_between_strokes(stroke_request,R)
            print "move to starting of stroke"
            output = request(go_to_stroke)

            # Send stroke
            print "writing stroke"    
            output = request(stroke_request)
            
            # clear stroke_request for next stroke
            stroke_request = waypoints()

    # After writing a letter, need to move over to next spot
    space_point = waypoints()
    p_end = data[-1]
    p_end = scale_factor * p_end
    p_end[1] = p_end[1] - .02
    p_end = numpy.dot(R,p_end)+pointc
    pointc = p_end
    space_point.points.append(make_point_from_array(p_end))
    go_to_stroke = move_between_strokes(space_point,R)
    output = request(go_to_stroke)

    return pointc

def move_between_strokes(stroke_request,R):
    go_to_stroke = waypoints()
    current_position = request_position()
    current_position = numpy.array([current_position.endpoint.x, current_position.endpoint.y, current_position.endpoint.z])
    raise_height = numpy.array([0, 0, .02])
    over_current = numpy.dot(R,raise_height) + current_position 
    start_point = stroke_request.points[0]
    start_point = numpy.array([start_point.x, start_point.y, start_point.z])
    over_start = numpy.dot(R,raise_height) + start_point

    go_to_stroke.points.append(make_point_from_array(current_position))
    go_to_stroke.points.append(make_point_from_array(over_current))
    go_to_stroke.points.append(make_point_from_array(over_start))
    go_to_stroke.points.append(make_point_from_array(start_point))
    return go_to_stroke


def get_ball_velocity(data):
    # This is a callback function for the ball_positions topic, everytime a new postion is published, the velocity is computed in this function
    # Right now, the position is in the image frame, need to change this to field or inertial frame
    global ball_pos
    global x_last
    global y_last
    global t_last
    global vx
    global vy
    x = data.x
    y = data.y
    t = data.t

    # store ball position as an array
    ball_pos = numpy.array(x,y,t)

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
    A1_length = .20      # m, length of A section closest to goal
    A2_length = .10      # m, length of A section closest to center
    B_length = .25       # m, length of B section
    field_width = .50    # m, short-side of field (only includes green area, exclude 2x4)
    half_field = A1_length + B_length + A2_length
    field_length = half_field*2
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
    if x > my_side[0] & x <= my_side[1]:
        ball_onside = 1         # ball is on our side
        # We can switch to offense now... maybe want to add a velocity checker to make sure the ball is not moving too quickly, or we might want to wait until the ball is essentially still
    else:
        ball_onside = 0         # ball is not on our side

    return ball_onside

if __name__ == "__main__":
    controller()

