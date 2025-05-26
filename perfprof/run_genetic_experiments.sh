#!/bin/bash

# Configurations
BUILD_DIR="../build"  
EXECUTABLE="$BUILD_DIR/tsp"
TIME_LIMIT=60 # seconds
SEED=0
NODES=1000
POP_SIZE_LIST=(10 25 50 100)  
NUM_INSTANCES=25
OUTPUT_FILE="results_genetic.csv"

# Check if the executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found!"
    exit 1
fi

# Create CSV header
echo -n "${#POP_SIZE_LIST[@]};" >> $OUTPUT_FILE
for PS in "${POP_SIZE_LIST[@]}"; do
    echo -n "genetic_${PS};" >> $OUTPUT_FILE
done
echo "" >> $OUTPUT_FILE

# Generate and run instances
for ((i=0; i<NUM_INSTANCES; i++)); do
    INSTANCE_FILE="instance_${i}.tsp"

    # Run VNS for each num_kicks value
    echo -n "inst${i};" >> $OUTPUT_FILE
    for PS in "${POP_SIZE_LIST[@]}"; do
        echo "Running Genetic on instance ${i} with pop_size=${PS} and seed=$((SEED + i)) ..."
        RESULT=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 10 -pop_size $PS | grep "Best cost" | awk '{print $3}')
        echo -n "${RESULT};" >> $OUTPUT_FILE
    done
    echo "" >> $OUTPUT_FILE
done

echo "Experiment completed. Results saved in $OUTPUT_FILE."
