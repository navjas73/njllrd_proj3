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

    while True:
        i = True


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
        
    elif rospy.get_param('/mode') == "draw":
        scale_factor = 0.01
        point1, point2, point3 = get_plane_points()
        points = waypoints()


        pointa = numpy.array([point1.endpoint.x, point1.endpoint.y, point1.endpoint.z])
        pointb = numpy.array([point2.endpoint.x, point2.endpoint.y, point2.endpoint.z])
        pointc = numpy.array([point3.endpoint.x, point3.endpoint.y, point3.endpoint.z])
        
        plane_vec = numpy.cross(pointa-pointb, pointb-pointc)

        plane_normal = plane_vec/numpy.linalg.norm(plane_vec)

        R = make_rotation_matrix(plane_normal)

        request = rospy.ServiceProxy('connect_waypoints', connect_waypoints)
        print "Draw mode. Type 'maze' or 'pyramids'."
        done = False
        while not done:
            letter = raw_input()
            if letter == 'end':
                print "I'm done"
                done = True
            elif letter == 'maze':
                data = maze()
                pointc = draw_letter(data,scale_factor*2,R,pointc)
                print "done drawing"
            elif letter == 'pyramids':
                data = pyramids()
                pointc = draw_letter(data,scale_factor,R,pointc)
                print "done drawing"
                '''elif letter.isalpha():    
                possibles = globals().copy()
                possibles.update(locals())
                method = possibles.get(letter)
                data = method()
                print data
                pointc = draw_letter(data,scale_factor,R,pointc)
                print "done drawing"'''
            else:
                print "Invalid input."


        # connect <0,0,0> and <.2,.2,0>
        #physical placement of 3 points on plane 
        # pt2            pt1
        # pt3

        #first_point = numpy.array([0,0,0])
        #second_point = numpy.array([.07,-.07,0])

        #first_point_rot = numpy.dot(R,first_point)
        #second_point_rot = numpy.dot(R,second_point)

        #first_point_rot_trans = first_point_rot + pointc
        #second_point_rot_trans = second_point_rot + pointc

        #points.points.append(make_point_from_array(first_point_rot_trans))
        #points.points.append(make_point_from_array(second_point_rot_trans))


    elif rospy.get_param('/mode')=="RRT":
        point1, point2 = get_connect_points()
        #goalstart = numpy.array([numpy.asarray(point1.config),numpy.asarray(point2.config)])
        rospy.wait_for_service('construct_RRT')
        request = rospy.ServiceProxy('construct_RRT', construct_RRT)
        output = request(point1.config, point2.config)  
        rospy.wait_for_service('connect_configs')
        request_config = rospy.ServiceProxy('connect_configs', connect_configs)
        success = request_config(output.path)   
        print success
        print output.path 

    elif rospy.get_param('/mode')=="BIRRT":
        point1, point2 = get_connect_points()
        #goalstart = numpy.array([numpy.asarray(point1.config),numpy.asarray(point2.config)])
        rospy.wait_for_service('construct_BIRRT')
        request = rospy.ServiceProxy('construct_BIRRT', construct_BIRRT)
        output = request(point1.config, point2.config)  
        rospy.wait_for_service('connect_configs')
        request_config = rospy.ServiceProxy('connect_configs', connect_configs)
        success = request_config(output.path)   
        print success
        print output.path 
    
    elif rospy.get_param('/mode') == "typewriter":
        scale_factor = 0.007
        done = False
        point1, point2, point3 = get_plane_points()
        points = waypoints()

        pointa = numpy.array([point1.endpoint.x, point1.endpoint.y, point1.endpoint.z])
        pointb = numpy.array([point2.endpoint.x, point2.endpoint.y, point2.endpoint.z])
        pointc = numpy.array([point3.endpoint.x, point3.endpoint.y, point3.endpoint.z])
        first_point_c = pointc
        plane_vec = numpy.cross(pointa-pointb, pointb-pointc)
        plane_normal = plane_vec/numpy.linalg.norm(plane_vec)
        R = make_rotation_matrix(plane_normal)

        request = rospy.ServiceProxy('connect_waypoints', connect_waypoints)
        print "Typewriter mode. Enter letters one at a time."

        while not done:
            letter = raw_input()
            print letter
            if letter == 'return':
                newline = numpy.array([-10,0,0])*scale_factor
                newline = numpy.dot(R,newline)
                pointc = first_point_c + newline
                first_point_c = pointc
                stroke = waypoints()
                stroke.points.append(make_point_from_array(pointc))
                go_to_stroke = move_between_strokes(stroke,R)
                output = request(go_to_stroke)
            elif letter == 'end':
                print "I'm done"
                done = True
            elif letter == 'space':
                newline = numpy.array([0,-10,0])*scale_factor
                newline = numpy.dot(R,newline)
                pointc = pointc + newline
                stroke = waypoints()
                stroke.points.append(make_point_from_array(pointc))
                go_to_stroke = move_between_strokes(stroke,R)
                output = request(go_to_stroke)
            elif len(letter) == 1:
                if letter.isalpha():    
                    possibles = globals().copy()
                    possibles.update(locals())
                    method = possibles.get(letter)
                    data = method()
                    print data
                    pointc = draw_letter(data,scale_factor,R,pointc)
                    print "Enter next letter"
                else:
                    print "Invalid input. Enter single lowercase letter or empty line for new line."
            else:
                print "Invalid input. Enter single lowercase letter or empty line for new line."

            '''if letter == 'end':
                done == True
            elif letter == 'return':
            	pointc[0] = first_point_c[0]-10*scale_factor
            	pointc[1] = first_point_c[1]
            elif letter.isalpha() and len(letter) < 2:
                possibles = globals().copy()
                possibles.update(locals())
                method = possibles.get(letter)
                data = method()
                print data
                pointc = draw_letter(data,scale_factor,R,pointc)
                print "Enter next letter" '''


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
    global x_last
    global y_last
    global t_last
    global vx
    global vy
    x = data.x
    y = data.y
    t = data.t

    dx = x-x_last
    dy = y-y_last
    dt = t-t_last
    vx = dx/dt
    vy = dy/dt

    x_last = x;
    y_last = y;
    t_last = t;
    ######## Since this is a callback function... need to either publish this velocity to a topic or change a global variable. For now use global var

def initialize_field():
    point1,point2,point3 = get_plane_points
    A1_length = .20      # m, length of A section closest to goal
    A2_length = .10      # m, length of A section closest to center
    B_length = .25       # m, length of B section
    field_width = .50    # m, short-side of field (only includes green area, exclude 2x4)
    half_field = A1_length + B_length + A2_length
    field_length = half_field*2
    # Create an array of positions relative to 0 (bottom left corner) that divide the field into sections
    field_divisions = numpy.array([0, A1_length, A1_length+B_length, half_field, half_field+A2_length, half_field + A2_length + B_length, field_length])

def get_ball_side(data,field_divisions):
    # Data = the ball position msg
    x = data.x  # this is in image coordinates, field divisions is in field coordinates... need a transform!!
    if arm == "left":
        my_side = numpy.array([field_divisions[1],field_divisions[3]]) 
    elif arm == "right":
        my_side = numpy.array([field_divisions[3],field_divisions[6]])

    # Check if ball is on our side of the field
    if x > my_side[0] & x <= my_side[1]:
        ball_onside = 1         # ball is on our side
        # We can switch to offense now... maybe want to add a velocity checker to make sure the ball is not moving too quickly, or we might want to wait until the ball is essentially still
    else:
        ball_onside = 0         # ball is not on our side


if __name__ == "__main__":
    controller()

