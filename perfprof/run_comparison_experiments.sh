#!/bin/bash

# Configurations
BUILD_DIR="../build"  
EXECUTABLE="$BUILD_DIR/tsp"
TIME_LIMIT=60 # seconds
SEED=50
NODES=1000
NUM_INSTANCES=50
OUTPUT_FILE="results_comparison.csv"

# Check if the executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found!"
    exit 1
fi

# Create CSV header
echo "2;vns;tabu" > $OUTPUT_FILE

# Generate and run instances
for ((i=0; i<NUM_INSTANCES; i++)); do
    INSTANCE_FILE="instance_${i}.tsp"
    
    echo "Running instance ${i}..."
    
    # Run VNS algorithm
    RESULT_VNS=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 3 | grep "Best cost" | awk '{print $3}')
    
    # Run Tabu algorithm
    RESULT_TABU=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 4 | grep "Best cost" | awk '{print $3}')
    
    # Save results
    echo "inst${i};${RESULT_VNS};${RESULT_TABU}" >> $OUTPUT_FILE

done

echo "Experiment completed. Results saved in $OUTPUT_FILE."
