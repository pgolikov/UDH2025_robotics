# UDH2025_robotics
Open repository to store materials for the Advanced Sensing, Robotics and IoT challenge at the 2nd Upstream Digital Hackathon

## Hackathon Task Description: Mission Planning for Swarm of Seismic Drones

The objective of this hackathon is to develop a mission planning system for a swarm of seismic drones. Participants will be tasked with creating algorithms that enable the drones to efficiently navigate and conduct seismic surveys over a designated area. The focus will be on optimizing flight paths based on terrain data, while also ensuring safety and coordination among the drones.

## Project Requirements:
* _Data Synchronization:_ Ensure that the swarm can aggregate seismic survey data efficiently, with minimal communication overhead and maximum data consistency;
* _Mission Planning System:_ Develop an autonomous system capable of generating mission plans for a swarm of drones, considering specific seismic survey requirements, battery exchanges during the mission, and terrain considerations for path planning;
* _Path Planning:_ Create algorithms that will allow drones to plan their flight paths based on terrain data. The algorithms must account for obstacles, elevation changes, and other environmental factors that may impact the drones’ flight and seismic exploration;
* _Scalability:_ Design solutions that work efficiently for both small swarms (5-20 drones) and larger swarms (20+ drones);
* _Performance Criteria:_ The solutions will be evaluated based on the following criteria:
    * _Efficiency:_ The total time taken to complete the seismic survey should be minimized.
    * _Path planner:_ The generated flight path must be short and optimized for energy consumption.
    * _Safety:_ There should be no collisions between drones or with environmental obstacles during the mission.

## Environment:
* Participants will utilize the Docker container with Gazebo simulation environment, ROS, Mavros, PX4-autopilot to test and validate their mission planning systems.
* Scripts for controlling a seismic drone will be provided.

## Input Data:
* The following input is provided: 
    * The mission planning system will utilize orthomosaic maps and Digital Elevation Models (DEM) in GeoTIFF format. It can be downloaded using [this link](https://www.dropbox.com/scl/fo/7wknt8y1o21nywsznn74d/AAxhbd2WGdAOrqduDDFYJns?rlkey=i6592rop9aja25suadp98kmb6&dl=0)
    * Preplan geometry of recording points on a regular 10x10m grid stored in ```./input_data/preplan.csv```.
    * Coordinates for the start area for the drone swarm and designated service areas for battery exchanges.

## Expected Deliverables: 

* _Algorithm/System Implementation:_ Code or software that can generate JSON file with mission for developed autonomous swarm control system. 
* _Simulation:_ A demo showing the swarm of drones navigating the map, avoiding obstacles, and collecting data.
* _Documentation:_ A detailed report or presentation describing your approach, methods, and any assumptions.

## Bonus Points: 
* Uzing ChatGPT or other LLM chats to generate flight missions and commands for swarm operations.
* Creative use of AI/ML models, RL, heuristics, or other techniques to solve complex challenges in mission planning.

