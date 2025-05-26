#!/bin/bash

# Configurations
BUILD_DIR="../build"  
EXECUTABLE="$BUILD_DIR/tsp"
TIME_LIMIT=60 # seconds
SEED=0
NODES=1000 
NUM_INSTANCES=50
OUTPUT_FILE="results_heuristics.csv"

# Check if the executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found!"
    exit 1
fi

# Create CSV header
echo "3;nearest neighbor;nearest neighbor + 2-opt; extra mileage + 2-opt" > $OUTPUT_FILE

# Generate and run instances
for ((i=0; i<NUM_INSTANCES; i++)); do

    INSTANCE_FILE="instance_${i}.tsp"
    
    RESULT_NN=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 1 | grep "Best cost" | awk '{print $3}')

    RESULT_NN_2OPT=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 2 | grep "Best cost" | awk '{print $3}')

    RESULT_EXTRA_MILEAGE=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 11 | grep "Best cost" | awk '{print $3}')

    echo "inst${i};${RESULT_NN};${RESULT_NN_2OPT};${RESULT_EXTRA_MILEAGE}" >> $OUTPUT_FILE
    
done

echo "Experiment completed. Results saved in $OUTPUT_FILE."
