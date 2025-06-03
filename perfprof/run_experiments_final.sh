#!/bin/bash

# Configurations
BUILD_DIR="../build"  
EXECUTABLE="$BUILD_DIR/tsp"
TIME_LIMIT=180 # seconds
SEED=150
NODES=1000
NUM_INSTANCES=50
OUTPUT_FILE="results_final.csv"

# Check if the executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found!"
    exit 1
fi

# Create CSV header
echo "3;Nearest Neighbor + 2-Opt;VNS;Hard-Fixing" > $OUTPUT_FILE

# Generate and run instances
for ((i=0; i<NUM_INSTANCES; i++)); do
    INSTANCE_FILE="instance_${i}.tsp"
    
    echo "Running instance ${i}..."
    
    # Run Nearest Neighbor + 2-Opt algorithm
    RESULT_NN_2OPT=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 2 | grep "Best cost" | awk '{print $3}')
    
    # Run VNS algorithm
    RESULT_VNS=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 3 | grep "Best cost" | awk '{print $3}')
    
    
    # Run Hard-Fixing 
    RESULT_HARD_FIXING=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 8 -mode 6 | grep "Best cost" | awk '{print $3}')

    # Save results
    echo "inst${i};${RESULT_NN_2OPT};${RESULT_VNS};${RESULT_HARD_FIXING}" >> $OUTPUT_FILE

done

echo "Experiment completed. Results saved in $OUTPUT_FILE."