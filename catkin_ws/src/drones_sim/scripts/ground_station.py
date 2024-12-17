#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import Waypoint 

class GroundStation(object):
    def __init__(self):
        self.command = 0
        self.param_command = Waypoint()
        self.dict_commands = {
                    'Test': 1,
                    'Drone: Arming': 2,
                    'Drone: Disarming': 3,
                    'Drone: Takeoff': 4,
                    'Drone: Land': 5
                    }

        self.publish_command = rospy.Publisher('/drone_commands_gs', Waypoint, queue_size=10)
        self.rate = rospy.Rate(10)        

    def moveByUser(self):
        try:
            for key in sorted(self.dict_commands, key=self.dict_commands.get):
                print(str(self.dict_commands[key]) + " - " + key)
            print('q - Quit from terminal')
            
            self.param_command = Waypoint()
            self.command = str(input("Write number of command: "))

            if self.command == 'q':
                raise SystemExit

            self.param_command.command = int(self.command)       

            if self.command == "4": #takeoff, set altitude
                self.param_command.z_alt = float(input("altitude: "))

            self.publish_command.publish(self.param_command) 

        except SyntaxError or ValueError:
            self.command=0


if __name__ == '__main__':
    try:
        rospy.init_node('ground_station', anonymous=True)
        node = GroundStation()
        while not rospy.is_shutdown():
            node.moveByUser()

    except rospy.ROSInterruptException:
        pass
