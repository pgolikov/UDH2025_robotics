#!/usr/bin/env python3

import rospy
from px4_drone import Drone
from mavros_msgs.msg import Waypoint


class DroneMission:
    def __init__(self):
        self.uav_name = str(rospy.get_param(rospy.get_name() + '/drone'))
        self.drone = Drone(self.uav_name)

        " Command variables "
        self.id_task_prev = None
        self.is_command_new = True

        " Mission executuion topic "
        rospy.Subscriber('/drone_commands_gs', Waypoint, self.callback_commands) #from ground_station.py
        self.rate = rospy.Rate(10)
   

    def callback_commands(self, msg):
        """ Callback function for getting data and commands form ground_station.py
        from topic '/drone_commands_gs' 

        Args:
            msg (Waypoint): message in Waypoint format.
        """        
        param_command = msg
        id_task = param_command.command
        rospy.logwarn('Get id task: ' + str(id_task))

        if self.id_task_prev != id_task:
            self.id_task_prev = id_task
            self.is_command_new = True

        if self.is_command_new == True:
               
            if id_task == 1:
                print("TEST vel_publisher")
                self.drone.pub_vel([1.0, 0.0, 0.0], yaw_rot=0)      

            if id_task == 2: #arm
                self.drone.arm()

            if id_task == 3: #disarm
                self.drone.dis_arm()

            if id_task == 4: #takeoff
                attitude = float(param_command.z_alt)
                self.drone.arm()
                self.drone.takeoff(height=attitude)

            if id_task == 5: #land
                self.drone.land()
                self.is_start_record_cords = False

            if id_task == 6:  # 'Go to local pos (in flight!)
                m = int(param_command.param1)
                point = [float(param_command.x_lat), float(
                    param_command.y_long), float(param_command.z_alt)]
                yaw_input = float(param_command.param2)
                self.drone.goTo(point, mode=m, yaw=yaw_input)

    def autonomous_mission(self):
        self.drone.arm()
        self.drone.takeoff(5.0)
        self.drone.hover(5.0)
        self.drone.land()


if __name__ == '__main__':
    rospy.sleep(4)
    rospy.init_node('drone_control', anonymous=True)

    mission = DroneMission()
    mission.autonomous_mission()

    while not rospy.is_shutdown():
        rospy.spin()
        
