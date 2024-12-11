#!/bin/bash

cd 
cd PX4-Autopilot

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
    X=$((COL * STEP))             # Calculate x-coordinate
    Y=$((ROW * STEP))             # Calculate y-coordinate
    
    tmux new-session -d -s "drone_$((n+1))" "PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"$X,$Y\" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i $((n+1))"
    sleep 5
done

echo "Launched $N PX4 instances in a grid (${ROWS} rows, ${COLS} columns) with a step of $STEP."

# roslaunch px4_contol px4_sim_multi.launch n_drones:$N