#!/usr/bin/env python
import rospy
import time
import math
import numpy
import ast
from njllrd_proj2.srv import *
from njllrd_proj2.msg import *
from std_msgs.msg import String

flag = False

def controller():
    rospy.init_node('controller')
    rospy.wait_for_service('request_endpoint')
    rospy.wait_for_service('request_orientation')
    rospy.Subscriber('user_input', String, handle_user_input)
    

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
https://github.com/navjas73/njllrd_proj3.git
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



def a():
	s_1 = numpy.array([[5,-3,0],[5,-7,0]])
	s_2 = numpy.array([[1,-1,0],[9,-5,0],[1,-9,0]])
	p_end = numpy.array([1,-9,0])
	return s_1,s_2,p_end

def b():
	s_1 = numpy.array([[5,-1,0],[5,-5,0]])
	s_2 = numpy.array([[1,-6,0],[1,-1,0],[5,-1,0],[9,-1,0],[9,-5,0],[5,-5,0],[5,-6,0],[1,-6,0]])
	p_end = numpy.array([1,-6,0])
	return s_1,s_2,p_end
	
def c():
	s_1 = numpy.array([[9,-8,0],[9,-1,0],[1,-1,0],[1,-8,0]])
	p_end = numpy.array([1,-8,0])
	return s_1,p_end

def d():
	s_1 = numpy.array([[3,-8,0],[1,-1,0],[9,-1,0],[7,-8,0],[3,-8,0]])
	p_end = numpy.array([1,-8,0])
	return s_1,p_end

def e():
	s_1 = numpy.array([[5,-1,0],[5,-6,0]])
	s_2 = numpy.array([[9,-6,0],[9,-1,0],[5,-1,0],[1,-1,0],[1,-6,0]])
	p_end = numpy.array([1,-6,0])
	return s_1,s_2,p_end
	
def f():
	s_1 = numpy.array([[5,-1,0],[5,-6,0]])
	s_2 = numpy.array([[9,-6,0],[9,-1,0],[5,-1,0],[1,-1,0]])
	p_end = numpy.array([1,-6,0])
	return s_1,s_2,p_end

def g():
	s_1 = numpy.array([[9,-9,0],[9,-1,0],[1,-1,0],[1,-9,0],[5,-9,0],[5,-5,0]])
	p_end = numpy.array([1,-9,0])
	return s_1,p_end

def h():
	s_1 = numpy.array([[1,-1,0],[9,-1,0]])
	s_2 = numpy.array([[5,-1,0],[5,-9,0]])
	s_3 = numpy.array([[9,-9,0],[1,-9,0]])
	p_end = numpy.array([1,-9,0])
	return s_1,s_2,s_3,p_end
	
def i():
	s_1 = numpy.array([[9,-1,0],[9,-5,0],[9,-9,0]])
	s_2 = numpy.array([[9,-5,0],[1,-5,0]])
	s_3 = numpy.array([[1,-1,0],[1,-5,0],[1,-9,0]])
	p_end = numpy.array([1,-9,0])
	return s_1,s_2,s_3,p_end

def j():
	s_1 = numpy.array([[9,-1,0],[9,-5,0],[9,-9,0]])
	s_2 = numpy.array([[9,-5,0],[1,-5,0]])
	s_3 = numpy.array([[1,-1,0],[1,-5,0]])
	p_end = numpy.array([1,-9,0])
	return s_1,s_2,s_3,p_end

def k():
	s_1 = numpy.array([[1,-1,0],[5,-1,0],[9,-1,0]])
	s_2 = numpy.array([[9,-7,0],[5,-1,0],[1,-7,0]])
	p_end = numpy.array([1,-7,0])
	return s_1,s_2,p_end
	
def l():
	s_1 = numpy.array([[9,-1,0],[1,-1,0],[1,-7,0]])
	p_end = numpy.array([1,-7,0])
	return s_1, p_end

def m():
	s_1 = numpy.array([[1,-1,0],[9,-1,0],[5,-5,0],[9,-9,0],[1,-9,0]])
	p_end = numpy.array([1,-9,0])
	return s_1, p_end
	
def n():
	s_1 = numpy.array([[1,-1,0],[9,-1,0],[1,-7,0],[9,-7,0]])
	p_end = numpy.array([1,-7,0])
	return s_1, p_end

def o():
	s_1 = numpy.array([[1,-8,0],[1,-1,0],[9,-1,0],[9,-9,0],[9,-8,0],[1,-8,0]])
	p_end = numpy.array([1,-8,0])
	return s_1, p_end

def p():
	s_1 = numpy.array([[5,-1,0],[5,-6,0],[9,-6,0],[9,-1,0],[5,-1,0],[1,-1,0]])
	p_end = numpy.array([1,-6,0])
	return s_1, p_end

def q():
	s_1 = numpy.array([[1,-7,0],[1,-1,0],[9,-1,0],[9,-7,0],[1,-7,0]])
	s_2 = numpy.array([[4,-4,0],[1,-8,0]])
	p_end = numpy.array([1,-8,0])
	return s_1, s_2, p_end

def r():
	s_1 = numpy.array([[5,-1,0],[5,-5,0]])
	s_2 = numpy.array([[1,-1,0],[5,-1,0],[9,-1,0],[9,-5,0],[5,-5,0],[5,-6,0],[1,-6,0]])
	p_end = numpy.array([1,-6,0])
	return s_1, s_2, p_end

def s():
	s_1 = numpy.array([[1,-1,0],[1,-6,0],[5,-6,0],[5,-1,0],[9,-1,0],[9,-5,0]])
	p_end = numpy.array([1,-6,0])
	return s_1, p_end
	
def t():
	s_1 = numpy.array([[9,-1,0],[9,-5,0],[9,-9,0]])
	s_2 = numpy.array([[9,-5,0],[1,-5,0]])
	p_end = numpy.array([1,-9,0])
	return s_1, s_2, p_end

def u():
	s_1 = numpy.array([[9,-1,0],[1,-1,0],[1,-7,0],[9,-7,0]])
	p_end = numpy.array([1,-7,0])
	return s_1, p_end

def v():
	s_1 = numpy.array([[9,-1,0],[1,-5,0],[9,-9,0]])
	p_end = numpy.array([1,-9,0])
	return s_1, p_end

def w():
    s_1 = numpy.array([[9,-1,0],[1,-3,0],[9,-5,0],[1,-7,0],[9,-9,0]])
    p_end = numpy.array([1,-9,0])
    return s_1, p_end
	
def x():
	s_1 = numpy.array([[1,-1,0],[9,-9,0]])
	s_2 = numpy.array([[9,-1,0],[1,-9,0]])
	p_end = numpy.array([1,-9,0])
	return s_1, s_2, p_end

def y():
	s_1 = numpy.array([[9,-1,0],[5,-5,0],[1,-5,0]])
	s_2 = numpy.array([[5,-5,0],[9,-9,0]])
	p_end = numpy.array([1,-9,0])
	return s_1, s_2, p_end

def z():
	s_1 = numpy.array([[5,-2,0],[5,-8,0]])
	s_2 = numpy.array([[9,-1,0],[9,-9,0],[1,-1,0],[1,-9,0]])
	p_end = numpy.array([1,-9,0])
	return s_1, s_2, p_end

def maze():
	s_1 = numpy.array([[7,-1,0],[7,-3,0],[9,-3,0],[9,-7,0],[3,-7,0],[3,-9,0]])
	s_2 = numpy.array([[3,-5,0],[3,-7,0]])
	s_3 = numpy.array([[1,-9,0],[1,-3,0],[5,-3,0],[5,-5,0],[7,-5,0]])
	s_4 = numpy.array([[5,-1,0],[5,-3,0]])
	s_5 = numpy.array([[6,-1,0],[6,-2,0]])
	s_6 = numpy.array([[6,-3,0],[6,-4,0]])
	s_7 = numpy.array([[7,-4,0],[8,-4,0]])
	s_8 = numpy.array([[8,-5,0],[8,-6,0]])
	s_9 = numpy.array([[7,-6,0],[6,-6,0]])
	s_10 = numpy.array([[5,-6,0],[4,-6,0]])
	s_11 = numpy.array([[4,-5,0],[4,-4,0]])
	s_12 = numpy.array([[3,-4,0],[2,-4,0]])
	s_13 = numpy.array([[2,-5,0],[2,-6,0]])
	s_14 = numpy.array([[2,-7,0],[2,-8,0]])
	p_end = numpy.array([1,-9,0])
	return s_1,s_2,s_3,s_4,s_5,s_6,s_7,s_8,s_9,s_10,s_11,s_12,s_13,s_14,p_end
	
def pyramids():
	s_1 = numpy.array([[0,0,0],[4,-3,0],[0,-7,0]])
	s_2 = numpy.array([[0,-1,0],[4,-3,0]])
	s_3 = numpy.array([[3,-4,0],[6,-7,0],[0,-13,0]])
	s_4 = numpy.array([[2,-5,0],[6,-7,0]])
	s_5 = numpy.array([[2,-11,0],[4,-13,0],[0,-17,0]])
	s_6 = numpy.array([[1,-12,0],[4,-13,0]])
	p_end = numpy.array([1,-17,0])
	return s_1,s_2,s_3,s_4,s_5,s_6,p_end
	
	
	
if __name__ == "__main__":
    controller()
