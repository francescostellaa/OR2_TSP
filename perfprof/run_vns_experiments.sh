#!/bin/bash

# Configurations
BUILD_DIR="../build"  
EXECUTABLE="$BUILD_DIR/tsp"
TIME_LIMIT=60 # seconds
SEED=0
NODES=1000
NUM_KICKS_LIST=(0 3 5 7 10 15)  
NUM_INSTANCES=50
OUTPUT_FILE="results_vns.csv"

# Check if the executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found!"
    exit 1
fi

# Create CSV header
echo -n "${#NUM_KICKS_LIST[@]};" >> $OUTPUT_FILE
for NK in "${NUM_KICKS_LIST[@]}"; do
    echo -n "vns_${NK};" >> $OUTPUT_FILE
done
echo "" >> $OUTPUT_FILE

# Generate and run instances
for ((i=0; i<NUM_INSTANCES; i++)); do
    INSTANCE_FILE="instance_${i}.tsp"

    # Run VNS for each num_kicks value
    echo -n "inst${i};" >> $OUTPUT_FILE
    for NK in "${NUM_KICKS_LIST[@]}"; do
        echo "Running VNS on instance ${i} with num_kicks=${NK} and seed=$((SEED + i)) ..."
        RESULT=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 3 -num_kicks $NK | grep "Best cost" | awk '{print $3}')
        echo -n "${RESULT};" >> $OUTPUT_FILE
    done
    echo "" >> $OUTPUT_FILE
done

echo "Experiment completed. Results saved in $OUTPUT_FILE."
