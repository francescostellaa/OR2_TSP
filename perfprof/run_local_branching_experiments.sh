#!/bin/bash

# Configurations
BUILD_DIR="../build"
EXECUTABLE="$BUILD_DIR/tsp"
TIME_LIMIT=180 # seconds
SEED=50
NODES=1000
NUM_INSTANCES=50
K_LIST=(10 20 30)
OUTPUT_FILE="results_local_branching.csv"

# Check if the executable exists and is executable
if [ ! -x "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found or not executable!"
    exit 1
fi

# Create CSV header
#echo "4;Local Branching 20;Local Branching 30;Local Branching 50;Local Branching 60" > "$OUTPUT_FILE"
echo "3;Local Branching 10;Local Branching 20;Local Branching 30" > "$OUTPUT_FILE"

# Generate and run instances
for ((i=0; i<NUM_INSTANCES; i++)); do

    INSTANCE_FILE="instance_${i}.tsp"

    echo "Running instance ${i}..."

    # Start CSV row
    echo -n "inst${i};" >> "$OUTPUT_FILE"

    for K in "${K_LIST[@]}"; do
        echo "Running Local Branching with k = $K on instance ${i}..."
        
        RESULT=$($EXECUTABLE -time_limit $TIME_LIMIT -seed $((SEED + i)) -nodes $NODES -alg 9 -mode 6 -k $K | grep "Best cost" | awk '{print $3}')
        echo -n "${RESULT};" >> "$OUTPUT_FILE"
    done

    echo "" >> "$OUTPUT_FILE"
done

echo "Experiment completed. Results saved in $OUTPUT_FILE."
