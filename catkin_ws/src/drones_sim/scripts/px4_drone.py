#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandHome
from tf.transformations import *

from sensor_msgs.msg import NavSatFix

from math import *
import numpy as np
from numpy.linalg import norm
import time
import os
import json

from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush, WaypointClear, WaypointSetCurrent
from geographic_msgs.msg import GeoPointStamped

class Drone:
    def __init__(self):
        self.uav_name = str(rospy.get_param(rospy.get_name() + '/drone'))
        rospy.loginfo("INIT-" + self.uav_name + "-DRONE")
    
        self.pose = None
        self.yaw = 0
        self.sp = None
        self.hz = 10
        self.rate = rospy.Rate(self.hz)

        self.current_state = State()
        self.global_pose = None
        self.prev_request = None
        self.prev_state = None
        self.state = None

        self.set_origin_once = True
        self.origin_position = None

        self.setpoint_publisher = rospy.Publisher('/uav' + self.uav_name + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.set_gp_origin_pub = rospy.Publisher('/uav' + self.uav_name + '/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=10)

        self.arming_client = rospy.ServiceProxy('/uav' + self.uav_name + '/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/uav' + self.uav_name + '/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy('/uav' + self.uav_name + '/mavros/cmd/takeoff', CommandTOL)

        self.push_waypoints = rospy.ServiceProxy('/uav' + self.uav_name + '/mavros/mission/push', WaypointPush)
        self.clear_waypoints = rospy.ServiceProxy('/uav' + self.uav_name + '/mavros/mission/clear', WaypointClear)
        self.set_current_waypoint = rospy.ServiceProxy('/uav' + self.uav_name + '/mavros/mission/set_current', WaypointSetCurrent)        

        rospy.Subscriber('/uav' + self.uav_name + '/mavros/state', State, self.state_callback)
        rospy.Subscriber('/uav' + self.uav_name + '/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)
        rospy.Subscriber('/uav' + self.uav_name + '/mavros/global_position/global', NavSatFix, self.gps_callback)

        self.pathname_task_list = "/home/sim/UDH2025_robotics/catkin_ws/src/drones_sim/mission/"
        self.filename_task_list = 'task_list_ASAD' + self.uav_name + '.json'

        self.mission_path = self.read_mission_json()

    def read_mission_json(self):
        """ Function to read mission from Json file  

        Returns:
            mission_list (list[CTask, ...]): list with tasks
        """       
        print("file path and name:", self.pathname_task_list + self.filename_task_list)

        # mission_list = CTask_list()

        mission_paths = []

        " check is file exist "
        if os.path.isfile(self.pathname_task_list + self.filename_task_list):
            with open(self.pathname_task_list + self.filename_task_list) as json_file:
                data = json.load(json_file)

            for task in data['task_list']:
        
                if  int(task['code']) == 10:
                    path_task = []

                    for pose in task['poses']:
                         path_task.append([float(pose['lat']), float(pose['lon']), float(pose['alt'])])

                    mission_paths.append(path_task)

            " Log info about mission "
            rospy.logwarn("---Mission readed---")
            rospy.loginfo("n_tasks: " + str(len(mission_paths)))
            rospy.loginfo("Task id and code")
            
            for path in mission_paths:
                print(path)
        else:
            rospy.logwarn("---Mission not readed!!! Task list is not exist ---")

        return mission_paths
    
    def gps_callback(self, data):
        self.global_pose = data

        if self.set_origin_once and self.global_pose != None:
            self.origin_position = data
            self.set_origin(self.origin_position)
            self.set_origin_once = False
            
    
    def state_callback(self, state):
        self.current_state = state

    def drone_pose_callback(self, pose_msg):
        self.pose = np.array([ pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z ])

    def arm(self):
        for i in range(self.hz):
            self.publish_setpoint([0,0,-1])
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
            self.rate.sleep()

    @staticmethod
    def get_setpoint(x, y, z, yaw=np.pi/2):
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
    
    def publish_setpoint(self, sp, yaw=np.pi/2):
        setpoint = self.get_setpoint(sp[0], sp[1], sp[2], yaw)
        setpoint.header.stamp = rospy.Time.now()
        self.setpoint_publisher.publish(setpoint)

    def takeoff(self, height):
        print("Takeoff...")
        self.sp = self.pose
        while self.pose[2] < height:
            self.sp[2] += 0.07
            self.publish_setpoint(self.sp)
            self.rate.sleep()

    def set_origin(self, pose):

        # Wait for the publisher to initialize
        rospy.sleep(1)

        origin = GeoPointStamped()
        origin.position.latitude = pose.latitude
        origin.position.longitude = pose.longitude
        origin.position.altitude = pose.altitude

        rospy.loginfo("Setting custom gp origin position...")

        self.set_gp_origin_pub.publish(origin)

    def hover(self, t_hold):
        print('Position holding...')
        t0 = time.time()
        self.sp = self.pose
        while not rospy.is_shutdown():
            t = time.time()
            if t - t0 > t_hold and t_hold > 0: break
            # Update timestamp and publish sp 
            self.publish_setpoint(self.sp)
            self.rate.sleep()

    def land(self):
        print("Landing...")
        self.sp = self.pose
        while self.sp[2] > - 1.0:
            self.sp[2] -= 0.08
            self.publish_setpoint(self.sp)
            self.rate.sleep()
        self.stop()

    def stop(self):
        while self.current_state.armed or self.current_state.mode == "OFFBOARD":
            if self.current_state.armed:
                self.arming_client(False)
            if self.current_state.mode == "OFFBOARD":
                self.set_mode_client(base_mode=0, custom_mode="MANUAL")
            self.rate.sleep()

    def goTo(self, wp, mode='global', tol=0.05):
        if mode=='global':
            goal = wp
        elif mode=='relative':
            goal = self.pose + wp
        print("Going to a waypoint...")
        self.sp = self.pose
        while norm(goal - self.pose) > tol:
            n = (goal - self.sp) / norm(goal - self.sp)
            self.sp += 0.03 * n
            self.publish_setpoint(self.sp)
            self.rate.sleep()

    def set_mode(self, mode):
        """Sets the vehicle's mode."""
        if self.set_mode_client(custom_mode=mode).mode_sent:
            rospy.loginfo(f"{mode} mode enabled")
        else:
            rospy.logwarn(f"Failed to set {mode} mode!")

    def test(self):
        
        self.arm()

        self.takeoff(height=5.0)

        # self.hover(t_hold=10.0)

        self.send_gps_path()

        # self.land()

    def send_gps_path(self):
        try:
            # Clear existing waypoints
            self.clear_waypoints()
            waypoints = []

            is_first = True

            for cord in self.mission_path[0]:
                wp = Waypoint()
                wp.frame = 3  # Global coordinate frame
                wp.command = 16  # NAV_WAYPOINT command
                wp.is_current = is_first  # Set as the first waypoint
                is_first = False
                wp.autocontinue = True
                wp.x_lat = cord[0]
                wp.y_long = cord[1]
                wp.z_alt = cord[2]
                last_cord = cord
                waypoints.append(wp)

            # Last Waypoint (LAND command)
            wp = Waypoint()
            wp.frame = 3  # Global coordinate frame
            wp.command = 21  # NAV_LAND command
            wp.is_current = False
            wp.autocontinue = True
            wp.x_lat = last_cord[0]
            wp.y_long = last_cord[1]
            wp.z_alt = 0  # Altitude ignored for LAND
            waypoints.append(wp)

            # Push waypoints to PX4
            response = self.push_waypoints(start_index=0, waypoints=waypoints)

            if response.success:
                rospy.loginfo("Waypoints sent successfully!")
                rospy.sleep(1)

                self.set_mode("AUTO.MISSION")
            else:
                rospy.logerr("Failed to send waypoints.")

            # After finishing the mission, drone will set status to 'AUTO.LOITER' 

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node('drone_control', anonymous=True)
    try:
        control = Drone() 
        control.test()

        while not rospy.is_shutdown():
            rospy.spin()  # Keeps the node alive and processes callback

    except rospy.ROSInterruptException:
        pass