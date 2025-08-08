#!/bin/bash

# Function to kill all background processes
cleanup() {
    echo "Stopping all processes..."
    kill 0  # Kill all processes in the current process group
    exit 0
}

# Set up trap to catch Ctrl+C (SIGINT) and SIGTERM
trap cleanup SIGINT SIGTERM

# Navigate to the camaras directory
cd ~/camaras

# Run all Python scripts in parallel
echo "Starting all scripts... (Press Ctrl+C to stop all)"
python3 src/mic.py &
PID1=$!
python3 main_robust.py &
PID2=$!
python3 src/spinning.py &
PID3=$!
python3 detecciones.py &
PID4=$!

# Store PIDs for cleanup
PIDS="$PID1 $PID2 $PID3 $PID4"

echo "Started processes with PIDs: $PIDS"
echo "Press Ctrl+C to stop all processes"

# Wait for all background processes to complete
wait
