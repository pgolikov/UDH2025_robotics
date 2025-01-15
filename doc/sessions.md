### To Launch a PX4 Session in Docker. 

Make the run file of the simulation executable:
``` bash
cd UDH2025_robotics
chmod +x run_multiple.sh
```

Launch PX4 ./run_multiple.sh sessions in 'UDH2025_robotics' folder, where N is the number of drones, and STEP (def 5m) is the distance between drones in the start grid:
``` bash
./run_multiple.sh <N> <STEP>
```

Example for 3 drones:
``` bash
cd UDH2025_robotics
./run_multiple.sh 3
```

Then, launch PX4 Mavros nodes. Set the number of drones using the 'n' argument:
``` bash
roslaunch drones_sim px4_sim_recursed.launch n:=3
```


### Detach from the Session:

To list all PX4 sessions:
``` bash
sudo tmux list-sessions
```

To attach to a specific tmux session (e.g., drone_1, drone_2, ... drone_N), use the following command:
``` bash
sudo tmux attach-session -t drone_1
```

While inside the tmux session, press:

    Ctrl + b, then d

This detaches you from the session without closing it.

Stopping Simulations:

To terminate all instances, you can use:

```bash
pkill -f px4
```


### Remove ros logs

```bash
rm -rf ~/.ros
```