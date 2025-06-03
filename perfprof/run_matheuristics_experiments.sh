#!/bin/bash

# Configurations
BUILD_DIR="../build"  
EXECUTABLE="$BUILD_DIR/tsp"
TIME_LIMIT=180 # seconds
SEED=200
NODES=1000 
NUM_INSTANCES=50
OUTPUT_FILE="results_matheuristics.csv"

# Check if the executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found!"
    exit 1
fi

# Create CSV header
echo "2;Local Branching; Hard Fixing" > $OUTPUT_FILE

# Generate and run instances
for ((i=0; i<NUM_INSTANCES; i++)); do

    INSTANCE_FILE="instance_${i}.tsp"
    
    RESULT_LB=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 9 -mode 6 | grep "Best cost" | awk '{print $3}')

    RESULT_HF=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 8 -mode 6 | grep "Best cost" | awk '{print $3}')

    echo "inst${i};${RESULT_LB};${RESULT_HF}" >> $OUTPUT_FILE
    
done

echo "Experiment completed. Results saved in $OUTPUT_FILE."
