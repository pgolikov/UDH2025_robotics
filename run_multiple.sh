#!/bin/bash

cd
cd PX4-Autopilot

CENTER_SWARM_POSE_X=0
CENTER_SWARM_POSE_Y=-600
CENTER_SWARM_POSE_Z=18.5

# Default configurations
DEFAULT_STEP=5
DEFAULT_ROWS=4

# Parse arguments
N=${1:-1}              # Number of drones, default is 1
STEP=${2:-$DEFAULT_STEP}  # Grid step, default is 5
ROWS=$DEFAULT_ROWS       # Number of rows, default is 4

# Calculate columns based on N and ROWS
COLS=$(( (N + ROWS - 1) / ROWS ))  # Ceiling of N / ROWS

# Launch drones in a grid
for n in $(seq 0 $((N-1))); do
    ROW=$((n % ROWS))             # Determine row (y-coordinate)
    COL=$((n / ROWS))             # Determine column (x-coordinate)
    X=$((COL * STEP + CENTER_SWARM_POSE_X)) # Adjust x-coordinate
    Y=$((ROW * STEP + CENTER_SWARM_POSE_Y)) # Adjust y-coordinate
    Z=$CENTER_SWARM_POSE_Z                # Use fixed Z-coordinate
    
    sudo tmux new-session -d -s "drone_$((n+1))" "PX4_GZ_WORLD=wadibirk PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"$X,$Y,$Z\" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i $((n+1))"
    sleep 4
done

echo "Launched $N PX4 instances in a grid (${ROWS} rows, ${COLS} columns) with a step of $STEP."
echo "To attach to a specific tmux session (e.g., drone_1, drone_2, ... drone_N), use the following command:"
echo "sudo tmux attach-session -t drone_1"