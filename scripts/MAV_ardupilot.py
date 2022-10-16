#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, ParamSet
from mavros_msgs.srv import CommandTOLRequest, CommandLongRequest, CommandLong, CommandBoolRequest
from mavros_msgs.msg import State, ExtendedState, ParamValue

from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np

from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math

DEBUG = False

class MAV2():

    def __init__(self):
        ############ Attributes #################

        self.rate = rospy.Rate(60)
        self.drone_state = State()
        self.goal_pose = PoseStamped()
        self.drone_pose = PoseStamped()
        self.goal_vel = TwistStamped()
        self.drone_state = State()
        self.drone_extended_state = ExtendedState()

        ############# Services ##################

        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.land_srv = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.param_set_srv = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        self.command_client = rospy.ServiceProxy("mavros/cmd/command", CommandLong)

    
        ############### Publishers ##############
        self.local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 20)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',  TwistStamped, queue_size=5)

        ########## Subscribers ##################
        self.local_atual = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback, queue_size=10) 
        self.extended_state_sub = rospy.Subscriber('/mavros/extended_state', ExtendedState, self.extended_state_callback, queue_size=2)        
    
        service_timeout = 15
        try:
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            rospy.logerr("failed to connect to services")

        rospy.loginfo("Services are up")
        

        while self.drone_state.mode == "":
            pass
        rospy.loginfo("Subscribers are up")
       
    ########## Callback functions ###########
    
    def state_callback(self, state_data):
        self.drone_state = state_data
        
    def extended_state_callback(self, es_data):
        self.drone_extended_state = es_data
        #Values for referente self.drone_extended_state.landed_state
        #uint8 LANDED_STATE_UNDEFINED = 0
        #uint8 LANDED_STATE_ON_GROUND = 1
        #uint8 LANDED_STATE_IN_AIR = 2
        #uint8 LANDED_STATE_TAKEOFF = 3
        #uint8 LANDED_STATE_LANDING = 4
    
    def local_callback(self, data):
        self.drone_pose = data
    
    def wait4start(self):
        rospy.loginfo("Waiting for user to set mode to GUIDED...")
        while not rospy.is_shutdown() and self.drone_state.mode != "GUIDED":
            rospy.sleep(0.01)
        else:
            if self.drone_state.mode == "GUIDED":
                rospy.loginfo("Mode set to GUIDED. Starting Mission...")
                return
            else:
                rospy.logerr("Error starting mission!")
                return
            
    ###Set mode: PX4 mode - string, timeout (seconds) - int
    def set_mode(self, mode):
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        service_timeout = 15
        rospy.wait_for_service('/mavros/set_mode', service_timeout)
        while (self.drone_state.mode != mode ):        
            response = self.set_mode_srv(0, mode)
        return response
            
    def set_param(self, param_name, param_value):
        service_timeout = 15
        rospy.wait_for_service('/mavros/param/set', service_timeout)
        a = ParamValue()
        a.real = param_value        
        response = self.param_set_srv(param_name, a)
        return response.success

    def takeoff(self, height):
        rospy.loginfo("TAKING OFF...")
        self.set_mode("GUIDED")
        self.arm()
        rospy.wait_for_service('/mavros/cmd/takeoff', 10)

        rospy.sleep(1)
        response = self.takeoff_srv(altitude=height)
        
        if response.success:
            rospy.loginfo("Takeoff completed!")
            return
        else:
            rospy.loginfo("Takeoff failed!")
            return


    def hold(self, hold_time): # hold time in seconds
        x_init = self.drone_pose.pose.position.x
        y_init = self.drone_pose.pose.position.y
        z_init = self.drone_pose.pose.position.z
        now = rospy.get_rostime()
        rospy.loginfo("Goind on hold for " + str(hold_time) + " s")
        while not rospy.get_rostime() - now > rospy.Duration(secs=hold_time):
            self.set_position(x_init, y_init, z_init)
    
    ####### Goal Position and Velocity #########
    def set_position(self, x, y, z, yaw = None):
        self.goal_pose.pose.position.x = float(x)
        self.goal_pose.pose.position.y = float(y)
        self.goal_pose.pose.position.z = float(z)
        if yaw == None:
            self.goal_pose.pose.orientation = self.drone_pose.pose.orientation
        else:
            [self.goal_pose.pose.orientation.x, 
            self.goal_pose.pose.orientation.y, 
            self.goal_pose.pose.orientation.z,
            self.goal_pose.pose.orientation.w] = quaternion_from_euler(0,0,yaw) #roll,pitch,yaw
            #print("X: " + str(self.goal_pose)))

        while self.drone_state.mode != "GUIDED":
            self.local_position_pub.publish(self.goal_pose)
            self.set_mode("GUIDED")
        self.local_position_pub.publish(self.goal_pose)

    
    def go_to_local(self, goal_x, goal_y, goal_z, yaw = None, TOL=0.2):
        rospy.loginfo("Going towards local position: (" + str(goal_x) + ", " + str(goal_y) + ", " + str(goal_z) + "), with a yaw angle of: " + str(yaw))
        current_x = self.drone_pose.pose.position.x
        current_y = self.drone_pose.pose.position.y
        current_z = self.drone_pose.pose.position.z
        [_,_,current_yaw] = euler_from_quaternion([self.drone_pose.pose.orientation.x,self.drone_pose.pose.orientation.y,self.drone_pose.pose.orientation.z,self.drone_pose.pose.orientation.w])
        if yaw == None:
            yaw = current_yaw
        
        if yaw*current_yaw >= 0:
            yaw_diff = yaw - current_yaw
        else:
            yaw_diff = yaw + current_yaw
        while not rospy.is_shutdown() and (np.sqrt((goal_x - current_x  )**2 + (goal_y - current_y)**2 + (goal_z - current_z)**2) + (yaw_diff)**2) > TOL:
            current_x = self.drone_pose.pose.position.x
            current_y = self.drone_pose.pose.position.y
            current_z = self.drone_pose.pose.position.z
            [_,_,current_yaw] = euler_from_quaternion([self.drone_pose.pose.orientation.x,self.drone_pose.pose.orientation.y,self.drone_pose.pose.orientation.z,self.drone_pose.pose.orientation.w])
            self.set_position(goal_x, goal_y, goal_z, yaw)
            if yaw*current_yaw >= 0:
                yaw_diff = yaw - current_yaw
            else:
                yaw_diff = yaw + current_yaw
        rospy.loginfo("Arrived at requested position")

    
    def change_auto_speed(self, guided_vel): # Velocity of guided mode in m/s
        rospy.loginfo("Changing speed to " + str(guided_vel))
        return self.set_param('WPNAV_SPEED', float(guided_vel*100))


    def set_vel(self, x, y, z, yaw = 0):
        while self.drone_state.mode != "GUIDED":
            self.goal_vel.twist.linear.x = float(x)
            self.goal_vel.twist.linear.y = float(y)
            self.goal_vel.twist.linear.z = float(z)

            self.goal_vel.twist.angular.z = float(yaw)
            self.velocity_pub.publish(self.goal_vel)  
            self.set_mode("GUIDED")

        self.goal_vel.twist.linear.x = float(x)
        self.goal_vel.twist.linear.y = float(y)
        self.goal_vel.twist.linear.z = float(z)

        self.goal_vel.twist.angular.z = float(yaw)
        self.velocity_pub.publish(self.goal_vel)    


    def land(self):
        rospy.loginfo("Landing...")

        response = self.set_mode('LAND')
        return response
    ########## Arm #######

    def arm(self):
        service_timeout = 15
        rospy.loginfo('ARMING MAV') 
        rospy.wait_for_service('mavros/cmd/arming', service_timeout)
        self.arm_srv(True)
        while not self.drone_state.armed:
            response = self.arm_srv(True)
        rospy.loginfo('Drone is armed')
        return response
        

    ########## Disarm #######
    def disarm(self):
        service_timeout = 15
        rospy.loginfo('DISARMING MAV')
        rospy.wait_for_service('mavros/cmd/arming', service_timeout)
        self.arm_srv(False)
        while self.drone_state.armed:
            response = self.arm_srv(False)
        rospy.loginfo('Drone is disarmed')
        return response


if __name__ == '__main__':
    rospy.init_node('mavbase2')
    mav = MAV2()
    mav.takeoff(10)
    rospy.sleep(30)
    mav.change_auto_speed(10)
    mav.go_to_local(0,0,5, yaw=0)
    mav.go_to_local(0,0,5, yaw=1.57)
    mav.go_to_local(5,0,5)
    mav.land()

  

    
