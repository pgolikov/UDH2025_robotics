#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from geographic_msgs.msg import GeoPoseStamped

class Drone:
    def __init__(self):
        self.rate = rospy.Rate(20)
        self.uav_name = str(1)
        rospy.loginfo("INIT-" + self.uav_name + "-DRONE")

        self.current_state = State() # Current autopilot state
        self.current_pose = PoseStamped() 

        # Publishers
        self.local_pos_pub = rospy.Publisher('/uav' + self.uav_name + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.global_pos_pub = rospy.Publisher('/uav' + self.uav_name + '/mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)
        self.setpoint_publisher = rospy.Publisher('/uav' + self.uav_name + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy('/uav' + self.uav_name +'/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/uav' + self.uav_name +'/mavros/set_mode', SetMode, self.state_callback)

        # Subscribers
        rospy.Subscriber('/uav' + self.uav_name +'/mavros/state', State, self.state_callback)
        rospy.Subscriber('/uav' + self.uav_name +'/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)

        # Wait for MAVROS services
        rospy.wait_for_service('/uav' + self.uav_name + '/mavros/cmd/arming')
        rospy.wait_for_service('/uav' + self.uav_name + '/mavros/set_mode')

        # Ensure the FCU is connected
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo("Waiting for FCU connection...")
            self.rate.sleep()
        rospy.loginfo("FCU connected!")

        # Send a few setpoints to prepare for OFFBOARD mode
        rospy.loginfo("Sending initial setpoints...")
        self.publish_setpoints([0, 0, 1.0], 5)  # Publish setpoints for 5 seconds

        # Set OFFBOARD mode and arm the vehicle
        self.set_mode("OFFBOARD")
        rospy.loginfo("READY-" + self.uav_name + "-DRONE")


    # Callback to monitor UAV state
    def state_callback(self, msg):
        self.current_state = msg    

    def drone_pose_callback(self, msg):
        self.current_pose = msg

    def arm_vehicle(self):
        """Arms the vehicle."""
        if self.arming_client(value=True).success:
            rospy.loginfo("Vehicle armed!")
        else:
            rospy.logwarn("Failed to arm the vehicle!")

    def set_mode(self, mode):
        """Sets the vehicle's mode."""
        if self.set_mode_client(custom_mode=mode).mode_sent:
            rospy.loginfo(f"{mode} mode enabled")
        else:
            rospy.logwarn(f"Failed to set {mode} mode!")

    def publish_setpoints(self, pose, duration):
        """Publishes position setpoints for a specific duration."""
        rate = rospy.Rate(20)  # 20 Hz
        start_time = rospy.Time.now()

        point = PoseStamped()
        point.header = Header()
        point.pose.position.x = pose[0]
        point.pose.position.y = pose[1]
        point.pose.position.z = pose[2]

        while not rospy.is_shutdown() and rospy.Time.now() - start_time < rospy.Duration(duration):
            point.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(point)
            rate.sleep()

    def takeoff(self, altitude):
        """Handles the takeoff sequence."""
        rospy.loginfo("Taking off...")

        self.publish_setpoints([self.current_pose.pose.position.x, self.current_pose.pose.position.y, altitude], 10)  # Hover for 10 seconds

    # def move_to_local(self, x, y, z):
    #     pose = PoseStamped()
    #     pose.pose.position.x = x
    #     pose.pose.position.y = y
    #     pose.pose.position.z = z
        
    #     rate = rospy.Rate(10)  # 10 Hz
    #     for _ in range(50):  # Publish setpoints for 5 seconds
    #         print(self.current_pose)
    #         self.local_pos_pub.publish(pose)
    #         rate.sleep()

    #     rospy.loginfo(f"Moving to local position: x={x}, y={y}, z={z}")

    def move_to_local(self, x, y, z, tolerance=0.25):
        """
        Moves the drone to a specified local position and waits until it reaches the target within a given tolerance.

        Args:
            x (float): Target x position (meters).
            y (float): Target y position (meters).
            z (float): Target z position (meters).
            tolerance (float): Position tolerance (meters). Default is 0.2.
            timeout (float): Maximum time to wait (seconds). Default is 30.
        """
        target_pose = PoseStamped()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z

        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            # Publish the target position
            self.local_pos_pub.publish(target_pose)

            # Get the current position
            current_x = self.current_pose.pose.position.x
            current_y = self.current_pose.pose.position.y
            current_z = self.current_pose.pose.position.z

            # Calculate the Euclidean distance to the target
            distance = ((current_x - x) ** 2 + (current_y - y) ** 2 + (current_z - z) ** 2) ** 0.5

            # Check if within tolerance
            if distance <= tolerance:
                rospy.loginfo(f"Reached target position: x={x}, y={y}, z={z}")
                return

            self.rate.sleep()

    def move_to_global(self, latitude, longitude, altitude):
        geo_pose = GeoPoseStamped()
        geo_pose.pose.position.latitude = latitude
        geo_pose.pose.position.longitude = longitude
        geo_pose.pose.position.altitude = altitude

        rate = rospy.Rate(10)  # 10 Hz
        for _ in range(50):  # Publish setpoints for 5 seconds
            self.global_pos_pub.publish(geo_pose)
            print("pubished ", geo_pose)
            rate.sleep()

        rospy.loginfo(f"Moving to global position: lat={latitude}, lon={longitude}, alt={altitude}")


    def land_vehicle(self):
        """Initiates the landing sequence."""
        rospy.loginfo("Landing...")
        self.set_mode("AUTO.LAND")
        rate = rospy.Rate(10)
        rospy.loginfo("Waiting for disarm...")
        while not rospy.is_shutdown() and self.current_state.armed:
            rate.sleep()
        rospy.loginfo("Landed and disarmed!")

    def test(self):
        self.set_mode("OFFBOARD")
        self.arm_vehicle()

        # Takeoff and hover
        self.takeoff(altitude=5.0)

        # Move to local position (x,y,z)
        self.move_to_local(0.0, 0.0, 5.0)

        # Move to global poisiton (lat, lon)
        # move_to_global(global_pos_pub, 47.3979756, 8.5461693, 47 + 5) # home
        # move_to_global(global_pos_pub, 47.39799, 8.5461693, 5) # point 1

        # Land
        self.land_vehicle()

if __name__ == "__main__":
    rospy.init_node('drone_control', anonymous=True)
    try:
        control = Drone() 
        control.test()
    except rospy.ROSInterruptException:
        pass
