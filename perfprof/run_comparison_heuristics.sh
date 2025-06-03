#!/bin/bash

# Configurations
BUILD_DIR="/Users/francescostella/CLionProjects/OR2_TSP/cmake-build-debug"
EXECUTABLE="$BUILD_DIR/tsp"
TIME_LIMIT=180 # seconds
SEED=50
NODES=1000
NUM_INSTANCES=10
OUTPUT_FILE="results_comparison_heuristics.csv"

# Check if the executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found!"
    exit 1
fi

# Create CSV header
echo "3;VNS;Hard fixing; Local Branching" > $OUTPUT_FILE

# Generate and run instances
for ((i=0; i<NUM_INSTANCES; i++)); do
    INSTANCE_FILE="instance_${i}.tsp"

    echo "Running instance ${i}..."

    # Run VNS algorithm
    RESULT_VNS=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 3 | grep "Best cost" | awk '{print $3}')

    # Run Tabu algorithm
    RESULT_HARD_FIXING=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 8 -mode 6 -prob_hard_fixing 0.7| grep "Best cost" | awk '{print $3}')

    RESULT_LOCAL_BRANCHING=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 8 -mode 6 -k 10| grep "Best cost" | awk '{print $3}')

    # Save results
    echo "inst${i};${RESULT_VNS};${RESULT_HARD_FIXING};${RESULT_LOCAL_BRANCHING}" >> $OUTPUT_FILE

done

echo "Experiment completed. Results saved in $OUTPUT_FILE."