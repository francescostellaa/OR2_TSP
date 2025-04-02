#!/bin/bash

# Configurations
BUILD_DIR="../build"  
EXECUTABLE="$BUILD_DIR/tsp"
TIME_LIMIT=60 # seconds
SEED=0
NODES=1000
TENURE_INTERVAL_LIST=(75 100)  
TENURE_SCALE_LIST=(0.2 0.5 0.7)
NUM_INSTANCES=50
OUTPUT_FILE="results_tabu.csv"

# Check if the executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found!"
    exit 1
fi

# Create CSV header
echo -n "$(( ${#TENURE_INTERVAL_LIST[@]} * ${#TENURE_SCALE_LIST[@]} ));" >> $OUTPUT_FILE
for NK in "${TENURE_INTERVAL_LIST[@]}"; do
    for NS in "${TENURE_SCALE_LIST[@]}"; do
        echo -n "tabu_${NK}_${NS};" >> $OUTPUT_FILE
    done
done
echo "" >> $OUTPUT_FILE

# Generate and run instances
for ((i=0; i<NUM_INSTANCES; i++)); do
    INSTANCE_FILE="instance_${i}.tsp"

    # Run VNS for each tenure interval value
    echo -n "inst${i};" >> $OUTPUT_FILE
    for NK in "${TENURE_INTERVAL_LIST[@]}"; do
        for SCALE in "${TENURE_SCALE_LIST[@]}"; do
            echo "Running TABU on instance ${i} with interval_tenure=${NK}, tenure_scale=${SCALE}, and seed=$((SEED + i)) ..."        RESULT=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 4 -interval_tenure $NK | grep "Best cost" | awk '{print $3}')
            RESULT=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 4 -interval_tenure $NK -tenure_scaling $SCALE | grep "Best cost" | awk '{print $3}')
            echo -n "${RESULT};" >> $OUTPUT_FILE
        done
    done
    echo "" >> $OUTPUT_FILE
done

echo "Experiment completed. Results saved in $OUTPUT_FILE."
