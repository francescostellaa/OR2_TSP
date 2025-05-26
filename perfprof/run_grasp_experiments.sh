#!/bin/bash

# Configurations
#BUILD_DIR="../build"
BUILD_DIR="/Users/francescostella/CLionProjects/OR2_TSP/cmake-build-debug"
EXECUTABLE="$BUILD_DIR/tsp"
TIME_LIMIT=60 # seconds
SEED=50
NODES=1000
NUM_INSTANCES=50
PROB_LIST=(0.01 0.03 0.05 0.07 0.1)
OUTPUT_FILE="results_grasp.csv"

# Check if the executable exists and is executable
if [ ! -x "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found or not executable!"
    exit 1
fi

# Create CSV header
echo "5;p=0.01;p=0.03;p=0.05;p=0.07;p=0.1" > "$OUTPUT_FILE"

# Generate and run instances
for ((i=0; i<NUM_INSTANCES; i++)); do
    echo "Running instance ${i}..."

    # Start CSV row
    echo -n "inst${i};" >> "$OUTPUT_FILE"

    for P in "${PROB_LIST[@]}"; do
        CMD="$EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 5 -prob_grasp $P"
        echo "  [prob_grasp = $P] Running: $CMD"

        RESULT=$($CMD 2>&1 | tee /dev/stderr | grep "Best cost" | awk '{print $3}')
        echo -n "${RESULT};" >> "$OUTPUT_FILE"
    done

    echo "" >> "$OUTPUT_FILE"
done

echo "Experiment completed. Results saved in $OUTPUT_FILE."
