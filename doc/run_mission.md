### 1. Launch simulation.
``` bash
cd UDH2025_robotics
./run_multiple.sh 4
```

### 2. Run evaluation.
``` bash
roslaunch mission_evaluation node.launch
```

### 3. Launch Mission.
``` bash
roslaunch drones_sim px4_sim_recursed.launch n:=4
```

