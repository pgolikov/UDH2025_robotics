#!/usr/bin/env python3

import rospy
import pandas as pd

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

from pyproj import Proj, Transformer
import math

import threading
import time



class Mission:
    def __init__(self):
        self.rate = rospy.Rate(20)
        self.CONDITION_DIST_REC_POINT = rospy.get_param('/CONDITION_DIST_REC_POINT')
        self.CONDITION_RECORDED_TIME = rospy.get_param('/CONDITION_RECORDED_TIME')
        self.SESMIC_SOURCE_PERIOD = rospy.get_param('/SESMIC_SOURCE_PERIOD')

        self.preplan_file = str(rospy.get_param('/preplan_file'))
        self.df = None
        self.read_recording_points()

        self.n_drones = 20
        self.drones_ids = range(1, self.n_drones+1)

        self.counter = 0
        self.positions = {f"uav{n}": (0,0, 0.0) for n in self.drones_ids}

        rospy.Subscriber('/recording_points', String, self.recording_callback)
        self.result_recording_pub = rospy.Publisher('/recording_result', String, queue_size=10)

        self.source_counter = 0
        self.source_status = 'wait'
        self.source_timer_pub = rospy.Publisher('/source_timer', String, queue_size=10)

        # Subscribers for each drone
        self.drone_coordinates = []
        for drone_id in self.drones_ids:
            topic_name = f"/uav{drone_id}/mavros/global_position/global"
            rospy.Subscriber(topic_name, NavSatFix, self.drone_poses_callback, callback_args=f"uav{drone_id}")

        # Init charging_stations
        self.charging_stations_file = str(rospy.get_param('/charging_stations_file'))
        self.df_charging = None
        self.read_charging_stations()
        rospy.Subscriber('/charging_stations', String, self.charging_stations_callback)
        self.charging_stations_pub = rospy.Publisher('/charging_result', String, queue_size=10)

        self.start_thread()


    # Function to start the thread
    def start_thread(self):
        self.thread = threading.Thread(target=self.update_value)
        self.thread.daemon = True  # Makes the thread exit when the main program exits
        self.thread.start()

    # Function to update the dataframe, time, source counter
    def update_value(self):
        while True:
            # Check if source ready
            if self.source_counter == self.SESMIC_SOURCE_PERIOD:
                for i in range(len(self.df)):
                    if self.df.at[i, 'status'] == 'wait_source':
                        self.df.at[i, 'status'] = "recording"
                self.source_counter = 0

            # Do recording
            for i in range(len(self.df)):
                if self.df.at[i, 'status'] == 'recording':
                    self.df.at[i, 'recorded_time'] += 1  # Update the 'recording_time' column with seconds
                    
                    if self.df.at[i, 'recorded_time'] < self.CONDITION_RECORDED_TIME:
                        self.result_recording_pub.publish(str(self.df.iloc[i].tolist()))

                    if self.df.at[i, 'recorded_time'] >= self.CONDITION_RECORDED_TIME and self.df.at[i, 'status'] == "recording":
                        self.df.at[i, 'status'] = "done"
                        self.df.at[i, 'is_recorded'] = True
                        self.result_recording_pub.publish(str(self.df.iloc[i].tolist()))

            print(self.df)
            print("Source_counter:", self.source_counter)
            self.source_timer_pub.publish(str(self.source_counter))

            time.sleep(1)  # Sleep for 1 second

            self.counter += 1
            self.source_counter += 1

    def drone_poses_callback(self, msg, drone_id):
        self.positions[drone_id] = (msg.latitude, msg.longitude)

    def charging_stations_callback(self, msg):
        drone_id, armed = msg.data.split(',')
        print(drone_id, armed)

        for _, row in self.df_charging.iterrows():
            X,Y,R = row['X'],row['Y'],row['radius']

            charging_pose = self.utm_to_latlon(easting = X, northing = Y)
            uav_data = self.positions.get(f"uav{drone_id}")

            distance = self.haversine(charging_pose, uav_data)

            if distance <= R and armed == "False": 
                print("CHARGING DRONE: " + str(drone_id))
                
                self.charging_stations_pub.publish(str(str(drone_id) + ',' + "OK"))
            
            else:
                print("CAN'T CHARG DRONE: " + str(drone_id))
                self.charging_stations_pub.publish(str(str(drone_id) + ',' + "NOT"))

    def recording_callback(self, msg):
        recstat, recline, drone_id = msg.data.split(',')
        
        target_row = self.df[(self.df['RECSTAT'] == int(recstat)) & (self.df['RECLINE'] == int(recline))]
        target_row_index = target_row.index[0]

        # Check if any rows are found
        if not target_row.empty:
            # print("Found row(s): / id in df:", target_row_index)
            # print(target_row)
            
            target_pose = self.utm_to_latlon(easting = target_row.iloc[0]['X'], northing = target_row.iloc[0]['Y'])
            uav_data = self.positions.get(f"uav{drone_id}")

            # print("target_pose", target_pose)
            # print("uav_data", uav_data)

            distance = self.haversine(target_pose, uav_data)
            # print("distance:", round(distance, 2), " m")

            if distance <= self.CONDITION_DIST_REC_POINT: 
                self.df.at[target_row_index, 'drone_id'] = 'uav' + drone_id
                self.df.at[target_row_index, 'status'] = "wait_source"               

        else:
            print("No matching row found.")

    def read_recording_points(self):
        self.df = pd.read_csv(self.preplan_file)
        self.df['status'] = 'none' # none, wait_source, recording, done
        self.df['is_recorded'] = False
        self.df['recorded_time'] = 0
        self.df['drone_id'] = None
        print(self.df)

    def read_charging_stations(self):
        self.df_charging = pd.read_csv(self.charging_stations_file)
        print(self.df_charging)

    def utm_to_latlon(self, easting, northing, zone_number=38, zone_letter="R"):
        proj_utm = Proj(proj='utm', zone=zone_number, south=(zone_letter and zone_letter.upper() < 'N'))
        transformer = Transformer.from_proj(proj_utm, Proj(proj="latlong", datum="WGS84"))
        lon, lat = transformer.transform(easting, northing)
        return float(lat), float(lon)

    def haversine(self, p1, p2):
        # Radius of the Earth in kilometers
        R = 6371.0

        # Convert latitude and longitude from degrees to radians
        lat1_rad = math.radians(p1[0])
        lon1_rad = math.radians(p1[1])
        lat2_rad = math.radians(p2[0])
        lon2_rad = math.radians(p2[1])

        # Calculate the differences
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        # Haversine formula
        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Distance
        distance = R * c * 1000 #to meters
        return distance

if __name__ == "__main__":
    rospy.init_node('mission', anonymous=True)
    try:
        control = Mission() 

        rospy.loginfo("Evaluation started!, Good Luck!")

        while not rospy.is_shutdown():

            rospy.spin()

    except rospy.ROSInterruptException:
        pass