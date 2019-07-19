#!/usr/bin/env python
#author: qinlin@andrew.cmu.edu
#import rospy
#from std_msgs.msg import String
import os
import subprocess
from PIL import Image
from mpmath import *
import copy
from interval import interval, inf, imath

#amankh
import rospy
import ackermann_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg

class state:
    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.psi = 0

class waypoint:
    def __init__(self):
        self.x_d = 0
        self.y_d = 0
        self.v_d = 0

class action:
    def __init__(self):
        self.speed = 0    

current_state = state()
current_waypoint = waypoint()
proposed_action = action()

def stateCallback(state_t):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
    current_state.pos_x = state_t.pose.pose.position.x
    current_state.pos_y = state_t.pose.pose.position.y
    current_state.pos_z = state_t.pose.pose.position.z
    current_state.psi = twist.twist.angular.z  # yaw_rate
    
    q = state_t.pose.pose.orientation   # orientation as quaternion
    x_dot = state_t.twist.twist.linear.x
    y_dot = state_t.twist.twist.linear.y
    yaw_dot = state_t.twist.twist.angular.z


    

def actionCallback(action_t):
    #proposed_action.clear();
    #ROS_INFO("Received action %f %f\n", msg->drive.speed, msg->drive.steering_angle)
    proposed_action.speed = action_t.drive.speed
    #proposed_action.push_back(msg->drive.steering_angle)

def waypointCallback(waypoint_t):
    current_waypoint.x_d = waypoint_t.x
    current_waypoint.y_d = waypoint_t.y
    waypoint_curvature = waypoint_t.z

def writeFlowstarFile():
    '''
        Function: Write target point: x_d, y_d, target velocity: v_d, and initial state of auxiliary variable into the template of flow* file
        Input: none
        Output: new flow* file
    '''
    filename = 'flowstar-2.1.0/model/RC_kinematic_purepursuit_template'
    with open(filename+'.model') as f:
        file_str = f.read()
    file_str = file_str.replace('X_D', str(current_waypoint.x_d))
    file_str = file_str.replace('Y_D', str(current_waypoint.y_d))
    file_str = file_str.replace('V_D', str(current_waypoint.v_d))
    av0 = initial_state_interval()
    file_str = file_str.replace('AV0', str(av0[0][0]))
    file_str = file_str.replace('AV1', str(av0[0][1]))
    with open(filename+'_new.model', "w") as f:
        f.write(file_str)
def executeFlowstar():
    '''
        Function: Execute reachability computation by calling flow*
        Input: none
        Output: none
    '''
    os.system('cd flowstar-2.1.0;'+'./flowstar < '+'model/RC_kinematic_purepursuit_template_new.model')
def visulization():
    '''
        Function: Visualize reachability results
        Input: none
        Output: none
    '''    
    os.system('cd flowstar-2.1.0/outputs;'+'gnuplot "RC_kinematic_purepursuit.plt"')
    im = Image.open('flowstar-2.1.0/images/RC_kinematic_purepursuit.eps')
    im_rotate = im.rotate(-90)
    im_rotate.show()
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    ## from amankh

    rospy.init_node('reach_flow')
    # subscribers
    rospy.Subscriber("ekf_localization/odom", nav_msgs.msg.Odometry, stateCallback)
    rospy.Subscriber("aa_planner/commands", ackermann_msgs.msg.AckermannDriveStamped, actionCallback)
    rospy.Subscriber("aa_planner/waypoints", geometry_msgs.msg.Point, waypointCallback)

    #publishers


    #spin
    rospy.spin()

def initial_state_interval():
    v_0 = interval[1.0, 1.01]
    pos_x_0 = interval[0.3, 0.31]
    pos_y_0 = interval[0.3, 0.31]
    psi_0 = interval[0, 0.01]
    x_d_0 = interval[1, 1]
    y_d_0 = interval[1, 1]
    av = (lambda v, psi, x_d, y_d, pos_x, pos_y: (-1 * v * imath.sinpi(psi) * x_d + y_d * v * imath.cospi(psi) - pos_y * v * imath.cospi(psi))/((x_d - pos_x)**2+(y_d - pos_y)**2))\
    (v_0, psi_0, x_d_0, y_d_0, pos_x_0, pos_y_0)
    return av
if __name__ == '__main__':
    writeFlowstarFile()
    executeFlowstar()
    visulization()