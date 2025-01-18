# Using ROS1 to Control Drones in a Gazebo Simulation

This guide explains how to control multiple drones simulated in Gazebo using ROS.

---

## 1. Launch the Simulation

Start the simulation with the following command:

```bash
roslaunch drones_sim px4_sim_recursed.launch n:=3
```

This command initializes a simulation with three drones. Each simulated PX4 drone has its MAVROS topics structured as:

```bash
/uav1/mavros/...
/uav2/mavros/...
/uav3/mavros/...
/uav4/mavros/...
```

The px4_sim_recursed.launch file also launches the px4_drones.py script, which subscribes to these topics and provides basic drone control commands.

## 2. Basic Drone Commands

The px4_drones.py script includes essential PX4 commands such as:

- arm: Arms the drone for flight.
- takeoff: Initiates a vertical takeoff.
- hover: Maintains the drone at a fixed position.
- land: Safely lands the drone.
- send_gps_path: flight by gps waypoints trajectory and land in last waypoint. 


## 3. Monitor Drone Position

Each drone's global position is available within the Gazebo world. You can view and interact with this data as follows:
View Global Position

#### To check a drone's global position, use:

```bash
rostopic echo /uav4/mavros/global_position/global
```

#### Check Topic Format

To determine the message format of the topic, use:

```bash
rostopic type /uav4/mavros/global_position/global
```

Output:

```bash
sensor_msgs/NavSatFix
```

For more details about working with ROS topics, see the ROS Topic Tutorial.
http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics 

## 4. Subscribe to Drone Data in Python

To access drone data in your Python scripts, you can subscribe to the relevant ROS topics. Example: test_get_gps.py

## 5. Additional Resources

Detailed documentation on using MAVROS with PX4 drones.

https://docs.px4.io/main/en/ros/mavros_offboard_python.html

## 6. Sample Mission for 4 Drones Presented in the Test Function in px4_drones.py

/home/vs/drone_hack/demo/UDH2025_robotics/catkin_ws/src/drones_sim/mission - provides an example of a flight mission.
The *.json files are presented as examples only, to load latitude, longitude, and altitude paths for drone flights.