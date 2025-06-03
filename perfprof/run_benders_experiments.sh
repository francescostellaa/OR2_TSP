#!/bin/bash

# Configurations
BUILD_DIR="../build"  
EXECUTABLE="$BUILD_DIR/tsp"
TIME_LIMIT=60 # seconds
SEED=50
NODES=300
NUM_INSTANCES=50
OUTPUT_FILE="results_benders.csv"

# Check if the executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found!"
    exit 1
fi

# Create CSV header
echo "2;regular;warm start" > $OUTPUT_FILE

# Generate and run instances
for ((i=0; i<NUM_INSTANCES; i++)); do
    INSTANCE_FILE="instance_${i}.tsp"
    
    echo "Running instance ${i}..."
    
    # Run Branch and cut algorithm
    RESULT_BENDERS=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 7 | grep "Total execution time: " | awk '{print $4}')
    
    # Run Branch and cut with warm start algorithm
    RESULT_WARM_START=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 7 -mode 1 | grep "Total execution time: " | awk '{print $4}')

    # Save results
    echo "inst${i};${RESULT_BENDERS};${RESULT_WARM_START}" >> $OUTPUT_FILE

done

echo "Experiment completed. Results saved in $OUTPUT_FILE."