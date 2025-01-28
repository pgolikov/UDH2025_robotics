#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, Waypoint
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear, WaypointSetCurrent
from tf.transformations import *
from std_msgs.msg import String

from sensor_msgs.msg import NavSatFix

from math import *
import numpy as np
from numpy.linalg import norm
import time
import os
import json
import pandas as pd

from geographic_msgs.msg import GeoPointStamped

import threading
import time


class Battery:
    def __init__(self):
        self.time_full_time = rospy.get_param('/FLGIHT_TIME_PER_BAT')
        self.time = self.time_full_time
        self.armed = False
        self.drone_id = None
        self.is_active = True
        self.charge_pub = rospy.Publisher('/charging_stations', String, queue_size=1)
        rospy.Subscriber('/charging_result', String, self.charging_stations_callback)
        self.start_thread()

    def charging_stations_callback(self, msg):
        """
        Process the received message, extract the UAV number, and charging conditions.
        """
        try:
            message_data = msg.data.split(',')
            # print(message_data)
            uav_number = message_data[0]
            status = str(message_data[1])
            if str(self.drone_id) in uav_number:
                rospy.loginfo(f"Message for UAV {self.drone_id}: {msg.data}")
                if status == 'OK':
                    rospy.loginfo(f"Battery replaced for UAV {self.drone_id}!")
                    self.recharge()
                    print('battery:', self.time, 'sec')
        except Exception as e:
            rospy.logerr(f"Error processing message: {e}")


    def charge(self):
        rospy.loginfo("Ask to charge")
        msg = str(self.drone_id) + ',' + str(self.armed)
        self.charge_pub.publish(msg)
        
    def start_thread(self):
        self.thread = threading.Thread(target=self.update_value)
        self.thread.daemon = True
        self.thread.start()
    
    def check(self, state, id):
        self.armed = state.armed
        self.drone_id = int(id)
        return self.is_active

    def recharge(self):
        self.time = self.time_full_time
        self.is_active = True

    def update_value(self):
        # rospy.loginfo(str(self.time))
        while True:
            if self.armed and self.is_active:
                if self.time <= 0:
                    rospy.logwarn("The battery is low. Landing Drone!")
                    self.is_active = False
                self.time -= 1
                time.sleep(1)
                # print(self.time)

class Drone:
    def __init__(self):
        self.uav_name = str(rospy.get_param(rospy.get_name() + '/drone'))
        rospy.loginfo("INIT-" + self.uav_name + "-DRONE")
        self.CONDITION_RECORDED_TIME = rospy.get_param('/CONDITION_RECORDED_TIME')       

        self.battery = Battery()

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

        self.pathname_task_list = str(rospy.get_param('/pathname_task_list'))       
        self.filename_task_list = 'task_list_ASAD' + self.uav_name + '.json'

        self.mission_path = self.read_mission_json()

        # Example for recording data
        self.id_rec_point = 0
        self.df = None
        self.pathname_recording_list = str(rospy.get_param('/pathname_recording_list')) 
        self.filename_recording_list = 'preplan_ASAD' + self.uav_name + '.csv'
        self.read_recording_points()
        self.point_recording_pub = rospy.Publisher('/recording_points', String, queue_size=1)
        rospy.Subscriber('/recording_result', String, self.recording_result_callback)

        self.is_recording = False

    def recording_result_callback(self, msg):
        """
        Process the received message, extract the UAV number, and check conditions.
        """
        try:
            message_data = eval(msg.data)
            # Extract fields from the message
            uav_number = message_data[-1]  # 'uav3' is the last element
            status = message_data[4]       # status is the 5th element
            # Check if the message is for the given UAV ID
            if str(self.uav_name) in uav_number:
                rospy.loginfo(f"Message for UAV {self.uav_name}: {msg.data}")
                # Check if the status is 'done'
                if status == 'done':
                    rospy.loginfo(f"'done' status detected for UAV {self.uav_name}!")
                    self.is_recording = False
        except Exception as e:
            rospy.logerr(f"Error processing message: {e}")


    def start_recording(self):
        if self.id_rec_point < len(self.df):
            RECSTAT = self.df.at[self.id_rec_point, 'RECSTAT']
            RECLINE = self.df.at[self.id_rec_point, 'RECLINE']
            msg = str(RECSTAT) + ',' + str(RECLINE) + ',' + str(self.uav_name)
            self.point_recording_pub.publish(msg)
            self.id_rec_point += 1
            self.is_recording = True

    def read_recording_points(self):
        self.df = pd.read_csv(self.pathname_recording_list + self.filename_recording_list)
        print(self.df)

    def read_mission_json(self):  
        print("file path and name:", self.pathname_task_list + self.filename_task_list)
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
        def myhook():
            print("shutdown time!")

        self.current_state = state
        self.battery.check(state, self.uav_name)

        if not self.battery.is_active:
            self.set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
            rospy.on_shutdown(myhook)

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
            self.sp[2] += 0.1
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


    def gps_flgiht(self, mission):
        # Start drone
        self.arm()
        self.takeoff(height=5.0)

        # Clear existing waypoints
        self.clear_waypoints()
        waypoints = []

        is_first = True

        for cord in mission:
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

        # Push waypoints to PX4 and start AUTO mode
        response = self.push_waypoints(start_index=0, waypoints=waypoints)
        if response.success:
            rospy.loginfo("Waypoints sent successfully!")
            rospy.sleep(1)
            self.set_mode("AUTO.MISSION")
        else:
            rospy.logerr("Failed to send waypoints.")

        # Wait for finish flight and land
        rospy.loginfo("Wait for finish flight and land")
        while self.current_state.armed:
            self.rate.sleep()

        rospy.loginfo("Landed!")

    def data_recording(self):
        # Start data recording
        rospy.loginfo("Recording START")
        self.start_recording()
        while self.is_recording:
            # rospy.sleep(self.CONDITION_RECORDED_TIME + 2) # just wait some time or /
            # self.is_recording = False
            self.rate.sleep() # / or wait for topic callback
        rospy.loginfo("Recording END")

    def demo_mission(self):
        try:
            for mission in self.mission_path:
                self.gps_flgiht(mission)
                self.data_recording()
            self.battery.charge()
                
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node('drone_control', anonymous=True)
    try:
        control = Drone() 
        control.demo_mission()

        while not rospy.is_shutdown():
            rospy.spin()  # Keeps the node alive and processes callback

    except rospy.ROSInterruptException:
        pass