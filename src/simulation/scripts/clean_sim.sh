#!/bin/bash

# Clean AutoTech / Webots simulation runs, dangling processes, and stale pipes/logs
MODEL_ARCH="$1"
FIFO_DIR="/tmp/autotech/${MODEL_ARCH}"

# 1. Safety check: avoid deleting /tmp/autotech/ if $1 is empty
if [ -z "$MODEL_ARCH" ]; then
    echo "Error: Please specify the model architecture (e.g. ./clean.sh res)"
    exit 1
fi

SIM_RANKS_FILE="${FIFO_DIR}/simulationranks"

echo "Stopping simulation and training processes for model:" $MODEL_ARCH


# Terminate Python training and Webots processes
if [ -f "SIM_RANKS_FILE" ]; then
    for PID in $(cut -d ' ' -f 1 "$SIM_RANKS_FILE"); do
        if kill -0 "$PID" 2>/dev/null; then
            pkill -9 -P "$PID" 2>/dev/null
            kill -9 "$PID" 2>/dev/null
        fi
    done
fi

# 3. Clean up the FIFOs and temporary run directory
echo "Cleaning $FIFO_DIR..."
rm -rf "$FIFO_DIR"

echo "Done cleaning $MODEL_ARCH!"

