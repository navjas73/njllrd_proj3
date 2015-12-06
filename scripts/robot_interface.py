#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Header
import baxter_interface
import PyKDL
from baxter_pykdl import baxter_kinematics
import numpy
import random

from njllrd_proj2.srv import *
from njllrd_proj2.msg import *
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from time import sleep

limb = None
kinematics = None
joint_names = None
tol         = None
points = None
tool_length = .080
joint_limits = None
initial_orientation = None

def move_to_point(initial_point,point):
# if q_next in reachable_workspace 
#while goal is not reached
#get x0 from current position
#calculate desired velocity   v_des = (x_next - x0)/time_step 
# get pseudoinverse
# multiply pseudoinverse by desired velocity (J^*v_des) to get joint velocities
# set joint velocities with set_joint_velocities
    distTraveled = 0
    x_init = numpy.array([initial_point.x, initial_point.y, initial_point.z])
    x_goal  = numpy.array([point.x, point.y, point.z])
    #correct stuff for feedback
    #correct_vector = numpy.subtract(x_goal,x_init)
    correct_vector = x_goal-x_init
    correct_dist = numpy.linalg.norm(correct_vector)
    correct_vector = correct_vector/correct_dist
    #print "x_init"
    #print x_init
    #print "x_goal"
    #print x_goal 
    at_goal = False
    #vel_mag = 0.02
    vel_mag = .02
    kp = .3
    deltaT = 0
    x0last = x_init;
    sleep_time = .005
    delta_q = .02
    
    global initial_orientation
    #uncomment when you don't want to recalculate position every time
    #x0   = x_init
    
    time_initial = rospy.get_time();
    deltaT  =  0;
    while not at_goal:
        #limb.exit_control_mode()
        #recalculating position every time
        #comment when set position once
        x0   = limb.endpoint_pose()   # current pose
        x0orientation = x0['orientation']
        x0   = x0['position']

        x0rotmax = quaternion_to_rotation(x0orientation[0],x0orientation[1],x0orientation[2],x0orientation[3])
        # offset vector. Add rotated offset vector to x0 to get desired end effector position 
        offset_vector = numpy.array([0,0,tool_length])
        rotated_offset = numpy.dot(x0rotmax,offset_vector)

        x0 = numpy.array([x0.x+rotated_offset[0,0], x0.y+rotated_offset[0,1], x0.z+rotated_offset[0,2]])


        distTraveled = numpy.linalg.norm(x0-x_init)
        

        deltaT = rospy.get_time() - time_initial;
        correct_x = correct_vector*vel_mag*deltaT+x_init
        error = x0 - correct_x
        error = error*kp
        
        #dist = numpy.linalg.norm(numpy.subtract(x_goal,x0))
        dist = numpy.linalg.norm(x_goal-x0)
        #print distTraveled
        #print numpy.linalg.norm(x_goal-x_init)
        if distTraveled >= numpy.linalg.norm(x_goal - x_init):
            at_goal = True
            print "within tolerance"
            limb.exit_control_mode()
            break
        else:
            # check if x_goal in reachable workspace

            #uncomment for feedback stuff
            v_des = (((x_goal-x0)/dist*vel_mag - error))
            

            #v_des = (x_goal-x0)/dist*vel_mag
            v_des = numpy.append(v_des, [0.0,0.0,0.0]) # calculate desired velocity, zero angular velocities
            
            J = kinematics.jacobian()
            J_T = kinematics.jacobian_transpose()
            J_psuinv  = kinematics.jacobian_pseudo_inverse()
            #print "vdes"
            #print v_des
            initial_objective = numpy.sqrt(numpy.linalg.det(numpy.dot(J,J_T)))
            
            q0 = limb.joint_angles()
            

            for key, value in q0.iteritems():
                    if key == 'left_s0':
                        temp_value = value
                        q0[key] = value + delta_q
                        J = kinematics.jacobian(q0)
                        J_T = kinematics.jacobian_transpose(q0)
                        s0_objective = numpy.sqrt(numpy.linalg.det(numpy.dot(J,J_T)))
                        q0[key] = temp_value
                    elif key == 'left_s1':
                        temp_value = value
                        q0[key] = value + delta_q
                        J = kinematics.jacobian(q0)
                        J_T = kinematics.jacobian_transpose(q0)
                        s1_objective = numpy.sqrt(numpy.linalg.det(numpy.dot(J,J_T)))
                        q0[key] = temp_value
                    elif key == 'left_e0':
                        temp_value = value
                        q0[key] = value + delta_q
                        J = kinematics.jacobian(q0)
                        J_T = kinematics.jacobian_transpose(q0)
                        e0_objective = numpy.sqrt(numpy.linalg.det(numpy.dot(J,J_T)))
                        q0[key] = temp_value
                    elif key == 'left_e1':
                        temp_value = value
                        q0[key] = value + delta_q
                        J = kinematics.jacobian(q0)
                        J_T = kinematics.jacobian_transpose(q0)
                        e1_objective = numpy.sqrt(numpy.linalg.det(numpy.dot(J,J_T)))
                        q0[key] = temp_value
                    elif key == 'left_w0':
                        temp_value = value
                        q0[key] = value + delta_q
                        J = kinematics.jacobian(q0)
                        J_T = kinematics.jacobian_transpose(q0)
                        w0_objective = numpy.sqrt(numpy.linalg.det(numpy.dot(J,J_T)))
                        q0[key] = temp_value
                    elif key == 'left_w1':
                        temp_value = value
                        q0[key] = value + delta_q
                        J = kinematics.jacobian(q0)
                        J_T = kinematics.jacobian_transpose(q0)
                        w1_objective = numpy.sqrt(numpy.linalg.det(numpy.dot(J,J_T)))
                        q0[key] = temp_value
                    elif key == 'left_w2':
                        temp_value = value
                        q0[key] = value + delta_q
                        J = kinematics.jacobian(q0)
                        J_T = kinematics.jacobian_transpose(q0)
                        w2_objective = numpy.sqrt(numpy.linalg.det(numpy.dot(J,J_T)))
                        q0[key] = temp_value


            b = numpy.array([s0_objective, s1_objective, e0_objective, e1_objective, w0_objective, w1_objective, w2_objective])
            b = b-initial_objective
            b = b/delta_q
            b = b*.00001






           
            q_dot = numpy.dot(J_psuinv,v_des) + numpy.dot((numpy.identity(7)-numpy.dot(J_psuinv,J)),numpy.transpose(b))
            #print "first half"
            #print numpy.dot(J_psuinv,v_des)
            #print "second half"
            #print numpy.dot((numpy.identity(7)-numpy.dot(J_psuinv,J)),numpy.transpose(b))
            q_dot = q_dot.tolist()
            q_dot = q_dot[0]
            #print "qdot"
            #print q_dot
            
            # Previous lines are objective function

            # Following three commands are used when objective function is off
            #q_dot = numpy.dot(J_psuinv,v_des)
            #q_dot = q_dot.tolist()
            #q_dot = q_dot[0]
            #print "q_dot"
            #print q_dot

            joint_command = dict(zip(joint_names,q_dot))
            #print "joint_command"
            #print joint_command
            limb.set_joint_velocities(joint_command)
            '''print "joint velocities"
            print limb.joint_velocities()'''
            #did this to try to set the rate of setting commands... didn't work
            #sleep(sleep_time)
            #x0last = x0 
    return True

def move_to_initial_point(point):
    x0 = limb.endpoint_pose()

    
    x0orientation = x0['orientation']
       
    x0rotmax = quaternion_to_rotation(x0orientation[0],x0orientation[1],x0orientation[2],x0orientation[3])

    offset_vector = numpy.array([0,0,tool_length])
    rotated_offset = numpy.dot(x0rotmax,offset_vector)

    new_pose = numpy.array([point.x-rotated_offset[0,0], point.y-rotated_offset[0,1], point.z-rotated_offset[0,2]])

    new_pose     = limb.Point(new_pose[0], new_pose[1], new_pose[2])
    #print "current_pose"
    #print current_pose['orientation']
    print "new_pose"
    print new_pose
    joints       = request_kinematics(new_pose, x0orientation,'left')
    #print "joints"
    #print joints
    limb.move_to_joint_positions(joints)
    print "moved to initial point"
    return True

def command_handler(data):
    i = 0
    for point in data.points.points:
        if (i is not 0):
            print "move from point: "
            print data.points.points[i-1]
            print "to point: "
            print point
            x = move_to_point(data.points.points[i-1],point)
            time.sleep(.5)
        else:
        	x = move_to_initial_point(point)
        i = i+1
        time.sleep(.1)
    print "end of command handler"
    return True


def handle_request_endpoint(data):
    endpoint = limb.endpoint_pose() # add in offset for tool later
    endpoint_position = point()
    q = endpoint['orientation']
    #endpoint_rotation = PyKDL.Rotation.Quaternion(q[0],q[1],q[2],q[3])
    '''print "endpoint_rotation"
    print endpoint_rotation'''
    endpoint_position.x = endpoint['position'][0]
    endpoint_position.y = endpoint['position'][1]
    endpoint_position.z = endpoint['position'][2]
    '''print endpoint_position
    print "our rotation"'''
    our_rotation = quaternion_to_rotation(q[0],q[1],q[2],q[3])
    '''print our_rotation'''
    offset_vector = numpy.array([0,0,tool_length])
    rotated_offset = numpy.dot(our_rotation,offset_vector)
    '''print "rotated offset"
    print rotated_offset'''
    endpoint_position.x = endpoint_position.x + rotated_offset[0,0]
    #print rotated_offset[0,0]
    endpoint_position.y = endpoint_position.y + rotated_offset[0,1]
    endpoint_position.z = endpoint_position.z + rotated_offset[0,2]

    endpoint_position
    '''print "offset_vector"
    print offset_vector'''
    return endpoint_position

def handle_request_orientation(data):
    q = limb.joint_angles()
    q_current = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
    for key, value in q.iteritems():
        if key == 'left_s0':
            q_current[0] = value
        elif key == 'left_s1':
            q_current[1] = value
        elif key == 'left_e0':
            q_current[2] = value
        elif key == 'left_e1':
            q_current[3] = value
        elif key == 'left_w0':
            q_current[4] = value
        elif key == 'left_w1':
            q_current[5] = value
        elif key == 'left_w2':
            q_current[6] = value
    return q_current

def quaternion_to_rotation(qx,qy,qz,qw):
    rotation_matrix = numpy.matrix([[1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],[2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw],[2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2]])
 
    return rotation_matrix


def request_kinematics(position, quaternion, side):
    # rospy.init_node("rsdk_ik_service_client")
    ns    = "ExternalTools/" + side + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr   = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'right': PoseStamped(
            header = hdr,
            pose   = Pose(position,quaternion)
            ),
        'left': PoseStamped(
            header = hdr,
            pose   = Pose(position,quaternion)
            ),
    }
    ikreq.pose_stamp.append(poses[side])
    print ikreq
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
    if (resp.isValid[0]):
        #print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        return limb_joints
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
    return False

def handle_connect_configs(data):
    path = numpy.asarray(data.path)
    print path
    goal = numpy.asarray(path[-1].config)
    for i in range(0,len(path)-1):
        q = numpy.asarray(path[i].config)
        q_next = numpy.asarray(path[i+1].config)
        reached_point = reach_goal(q,q_next)
        #joint_command = dict(zip(joint_names,q_dot))
        #print "joint_command"
        #print joint_command
        #limb.set_joint_velocities(joint_command)

    return True

def reach_goal(start, goal):
    at_goal = 0 
    totDist = numpy.linalg.norm(goal-start)
    while not at_goal:
        q = limb.joint_angles()
        q_current = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        for key, value in q.iteritems():
            if key == 'left_s0':
                q_current[0] = value
            elif key == 'left_s1':
                q_current[1] = value
            elif key == 'left_e0':
                q_current[2] = value
            elif key == 'left_e1':
                q_current[3] = value
            elif key == 'left_w0':
                q_current[4] = value
            elif key == 'left_w1':
                q_current[5] = value
            elif key == 'left_w2':
                q_current[6] = value
        dist = numpy.linalg.norm(goal-q_current)
        distTraveled = numpy.linalg.norm(q_current-start)
        q_dot = (goal-q_current)/dist*0.1; 
        if distTraveled >= totDist:
            at_goal = True
            print "within tolerance"
            limb.exit_control_mode()
            break
        else:
            joint_command = dict(zip(joint_names,q_dot))
            limb.set_joint_velocities(joint_command)
    return True

def robot_interface():
    rospy.init_node('robot_interface')
    
    # Subscribes to waypoints from controller, sent in a set 
    z = rospy.Service('connect_waypoints', connect_waypoints, command_handler)

    # Sends message to controller when it's done
    rospy.Publisher('state', String, queue_size=10)
    
    # If requested, returns endpoint pose
    s = rospy.Service('request_endpoint', request_endpoint, handle_request_endpoint)

    a = rospy.Service('request_orientation', request_orientation, handle_request_orientation)

    k = rospy.Service('connect_configs', connect_configs, handle_connect_configs)

    global joint_limits
    global limb 
    global kinematics
    global joint_names
    global tol
    tol         = 0.01
    
    # Left limb
    joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
    limb        = baxter_interface.Limb('left') #instantiate limb
    kinematics  = baxter_kinematics('left')
    joint_limits = numpy.array([[-2.461, .890],[-2.147,1.047],[-3.028,3.028],[-.052,2.618],[-3.059,3.059],[-1.571,2.094],[-3.059,3.059]])
    max_joint_speeds = numpy.array([2.0,2.0,2.0,2.0,4.0,4.0,4.0])

    # Right limb
    #joint_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
    #limb = baxter_interface.Limb('right')
    #kinematics = baxter_kinematics('right')
    #joint_limits = numpy.array([[-2.461, .890],[-2.147,1.047],[-3.028,3.028],[-.052,2.618],[-3.059,3.059],[-1.571,2.094],[-3.059,3.059]])
    #max_joint_speeds = numpy.array([2.0,2.0,2.0,2.0,4.0,4.0,4.0])

    global points
    points = waypoints()
    rospy.spin()


if __name__ == "__main__":
    robot_interface()
    
