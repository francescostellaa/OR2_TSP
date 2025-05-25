#!/bin/bash

# Configurations
#BUILD_DIR="../build"
BUILD_DIR="../cmake-build-debug"
EXECUTABLE="$BUILD_DIR/tsp"
TIME_LIMIT=180 # seconds
SEED=50
NODES=1000
NUM_INSTANCES=10
PROB_LIST=(0.6 0.7 0.8)
OUTPUT_FILE="results_hard_fixing_high_prob.csv"

# Check if the executable exists and is executable
if [ ! -x "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found or not executable!"
    exit 1
fi

# Create CSV header
echo "3;Hard fixing 0.6;Hard fixing 0.7;Hard fixing 0.8" > "$OUTPUT_FILE"

# Generate and run instances
for ((i=0; i<NUM_INSTANCES; i++)); do
    echo "Running instance ${i}..."

    # Start CSV row
    echo -n "inst${i};" >> "$OUTPUT_FILE"

    for P in "${PROB_LIST[@]}"; do
        CMD="$EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 8 -mode 6 -prob_hard_fixing $P"
        echo "  [prob_hard_fixing = $P] Running: $CMD"

        RESULT=$($CMD 2>&1 | tee /dev/stderr | grep "Best cost" | awk '{print $3}')
        echo -n "${RESULT};" >> "$OUTPUT_FILE"
    done

    echo "" >> "$OUTPUT_FILE"
done

echo "Experiment completed. Results saved in $OUTPUT_FILE."
