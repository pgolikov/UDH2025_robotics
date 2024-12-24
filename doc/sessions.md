### To launch px4 session in docker.

Make run file of simulation executable:

``` bash
cd UDH2025_robotics
chmod +x run_multiple.sh
```

Launch px4 sessions, where N is number of drones, STEP is distance between drone in start grid.
``` bash
./run_multiple.sh <N> <STEP>

# example for 3 drones
./run_multiple.sh 3
```

To show all list of px4 sessions:
``` bash
sudo tmux list-sessions
```

To attach to a specific tmux session (e.g., drone_1, drone_2, ... drone_N), use the following command:

``` bash
sudo tmux attach-session -t drone_1
```

Then, launch px4 mavros nodes. Set number of drones in argumtn 'n'
``` bash
roslaunch drones_sim px4_sim_recursed.launch n:=3
```


Detach from the Session:

While inside the tmux session, press:

- Ctrl+b, then d

This detaches you from the session without closing it.

Stopping Simulations: To terminate all instances, you can use:

```bash
pkill -f px4
```