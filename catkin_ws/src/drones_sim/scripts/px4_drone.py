#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import *

import numpy as np
from numpy.linalg import norm
import time

class Drone:
    def __init__(self, drone_id):

        self.uav_name = str(drone_id)
        rospy.logwarn("INIT-" + self.uav_name + "-DRONE")

        self.hz = 10
        self.rate = rospy.Rate(self.hz)

        self.current_state = State() # Current autopilot state
        self.pose = None # current drone pose
        self.current_status = "Init" # Drone state: Init, Arming, TakeOff, Hover, Landing, Grounded, Recording

        self.setpoint_publisher = rospy.Publisher('/uav' + self.uav_name + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy('/uav' + self.uav_name +'/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/uav' + self.uav_name +'/mavros/set_mode', SetMode, self.state_callback)

        rospy.Subscriber('/uav' + self.uav_name +'/mavros/state', State, self.state_callback)
        rospy.Subscriber('/uav' + self.uav_name +'/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)
        

    # --- CALLBACK FUNCTIONS ---
    def state_callback(self, state):
        self.current_state = state

    def drone_pose_callback(self, pose_msg):
        self.pose = np.array([ pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z ])

    # --- DRONE CONTROL FUNCTIONS ---
    def arm(self): # OK
        self.current_status = 'Arming'
        rospy.logwarn(self.uav_name + self.current_status)

        for i in range(self.hz):
            self.rate.sleep()
    
        # wait for FCU connection
        while not self.current_state.connected:
            print('Waiting for FCU connection...')
            self.rate.sleep()

        prev_request = rospy.get_time()
        prev_state = self.current_state
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if self.current_state.mode != "OFFBOARD" and (now - prev_request > 2.):
                self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                prev_request = now 
            else:
                if not self.current_state.armed and (now - prev_request > 2.):
                    self.arming_client(True)
                    prev_request = now 

            # older versions of PX4 always return success==True, so better to check Status instead
            if prev_state.armed != self.current_state.armed:
                print("Vehicle armed: %r" % self.current_state.armed)

            if prev_state.mode != self.current_state.mode: 
                print("Current mode: %s" % self.current_state.mode)
            prev_state = self.current_state

            if self.current_state.armed:
                break
            # Update timestamp and publish sp 
            self.publish_setpoint([0,0,-1])
            self.current_status = 'Armed'
            self.rate.sleep()

    def takeoff(self, height): # OK
        self.current_status = 'Takeoff'
        print(self.uav_name, " Takeoff...", self.pose)
        sp = self.pose
        while self.pose[2] < height:
            sp[2] += 0.05
            self.publish_setpoint(sp)
            self.rate.sleep()
        self.current_status = "Hover"
        print(self.uav_name, ' ', self.current_status)
        
    def hover(self, t_hold): # OK
        print('Position holding...')
        t0 = time.time()
        sp = self.pose
        while not rospy.is_shutdown():
            t = time.time()
            if t - t0 > t_hold and t_hold > 0: break
            # Update timestamp and publish sp 
            self.publish_setpoint(sp)
            self.rate.sleep()

    def land(self): # OK
        print(self.uav_name, " Landing...")
        self.current_status = "Landing"
        sp = self.pose
        while sp[2] > - 1.0:
            sp[2] -= 0.075
            self.publish_setpoint(sp)
            self.rate.sleep()
        # self.stop()
        self.current_status = "Grounded"
        print(self.uav_name, self.current_status)

    def dis_arm(self):
        if self.current_state.armed:
            rospy.loginfo("Disarming")
            rospy.wait_for_service('/mavros/cmd/arming')
            try:
                arming_cl = rospy.ServiceProxy(
                    '/mavros/cmd/arming', CommandBool)
                response = arming_cl(value=False)
                self.current_status = 'landed'
            except rospy.ServiceException as e:
                rospy.loginfo("Disarming failed: %s" % e)

    @staticmethod
    def get_setpoint(x, y, z, yaw=0): # early yaw=np.pi/2
        set_pose = PoseStamped()
        set_pose.pose.position.x = x
        set_pose.pose.position.y = y
        set_pose.pose.position.z = z
        
        q = quaternion_from_euler(0, 0, yaw)
        set_pose.pose.orientation.x = q[0]
        set_pose.pose.orientation.y = q[1]
        set_pose.pose.orientation.z = q[2]
        set_pose.pose.orientation.w = q[3]
        return set_pose
        
    def publish_setpoint(self, sp, yaw=0): #yaw=np.pi/2
        setpoint = self.get_setpoint(sp[0], sp[1], sp[2], yaw)
        setpoint.header.stamp = rospy.Time.now()
        self.setpoint_publisher.publish(setpoint)

    def goTo(self, wp, mode='global', tol=0.25):
        print("Current setpoint", wp)

        speed_coef = 0.25

        if mode=='global':
            goal = wp
        elif mode=='relative':
            goal = self.pose + wp

        print("Going to a waypoint...")
        sp = self.pose

        while norm(goal - self.pose) > tol:
            n = (goal - sp) / norm(goal - sp)
            sp += speed_coef * n
            self.publish_setpoint(sp)
            self.rate.sleep()