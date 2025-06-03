#!/bin/bash

# Configurations
BUILD_DIR="../build"  
EXECUTABLE="$BUILD_DIR/tsp"
TIME_LIMIT=60 # seconds
SEED=100
NODES=300
NUM_INSTANCES=50
OUTPUT_FILE="results_exact.csv"

# Check if the executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found!"
    exit 1
fi

# Create CSV header
echo "2;Benders;Branch-and-Cut" > $OUTPUT_FILE

# Generate and run instances
for ((i=0; i<NUM_INSTANCES; i++)); do
    INSTANCE_FILE="instance_${i}.tsp"
    
    echo "Running instance ${i}..."
    
    # Run Branch and cut algorithm
    RESULT_BENDERS=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 7 mode 1 | grep "Total execution time: " | awk '{print $4}')
    
    # Run Branch and cut with warm start algorithm
    RESULT_BC=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 6 -mode 6 | grep "Total execution time: " | awk '{print $4}')

    # Save results
    echo "inst${i};${RESULT_BENDERS};${RESULT_BC}" >> $OUTPUT_FILE

done

echo "Experiment completed. Results saved in $OUTPUT_FILE."