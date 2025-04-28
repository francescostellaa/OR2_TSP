#!/bin/bash

# Configurations
BUILD_DIR="../build"  
EXECUTABLE="$BUILD_DIR/tsp"
TIME_LIMIT=60 # seconds
SEED=50
NODES=200
NUM_INSTANCES=10
OUTPUT_FILE="results_bc.csv"

# Check if the executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found!"
    exit 1
fi

# Create CSV header
echo "3;b&c;b&c+warm_start;b&c+heuristic_posting" > $OUTPUT_FILE

# Generate and run instances
for ((i=0; i<NUM_INSTANCES; i++)); do
    INSTANCE_FILE="instance_${i}.tsp"
    
    echo "Running instance ${i}..."
    
    # Run Branch and cut algorithm
    RESULT_BC=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 6 | grep "Total execution time: " | awk '{print $4}')
    
    # Run Branch and cut with warm start algorithm
    RESULT_WARM_START=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 6 -mode 1 | grep "Total execution time: " | awk '{print $4}')
    
    # Run Branch and cut with warm start algorithm
    RESULT_POSTING=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 6 -mode 2 | grep "Total execution time: " | awk '{print $4}')
    
    # Save results
    echo "inst${i};${RESULT_BC};${RESULT_WARM_START};${RESULT_POSTING}" >> $OUTPUT_FILE

done

echo "Experiment completed. Results saved in $OUTPUT_FILE."