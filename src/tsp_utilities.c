#include <tsp_utilities.h>

/**
 * Print error message and exit the program
 * @param err error message
 */
void print_error(const char *err) { printf("\n\n ERROR: %s \n\n", err); fflush(NULL); exit(1); } 

double random01() { return ((double) rand() / RAND_MAX); }

/**
 * Generate random coordinates for the nodes of the TSP instance
 * @param inst instance to store the coordinates
 */
void random_instance_generator(instance *inst) {
    srand(inst->seed);
    inst->points = (point*) malloc(inst->nnodes * sizeof(point));
    inst->best_sol = (int *) malloc((inst->nnodes + 1) * sizeof(int));

    if ( VERBOSE >= 1000) { printf("Number of Nodes: %d\n", inst->nnodes); fflush(NULL); };
    for (int i = 0; i < inst->nnodes; i++){
        inst->points[i].x = random01() * MAX_BOUNDARY;
        inst->points[i].y = random01() * MAX_BOUNDARY;
    }
    
    return;
}

/**
 * Compute the Euclidean distance between two nodes
 * @param i index of the first node
 * @param j index of the second node
 * @param inst instance with the nodes
 * @return distance between the nodes
 */
double dist(int i, int j, instance *inst) {
    return (sqrt(pow(inst->points[i].x - inst->points[j].x, 2) + pow(inst->points[i].y - inst->points[j].y, 2)));
}

/**
 * Compute the squared Euclidean distance between two nodes
 * @param i index of the first node
 * @param j index of the second node
 * @param inst instance with the nodes
 * @return squared distance between the nodes
 */
double dist2(int i, int j, instance *inst) {
    return (pow(inst->points[i].x - inst->points[j].x, 2) + pow(inst->points[i].y - inst->points[j].y, 2));
}

/**
 * Compute the cost of all edges in the instance
 * @param instance instance with the nodes
 */
void compute_all_costs(instance* instance) {
    int n = instance->nnodes;
    instance->cost = (double*)malloc(n * n * sizeof(double));

    for (int i = 0; i < n; i++) {
        for (int j = i; j < n; j++) { 
            double d = dist(i, j, instance);
            instance->cost[i * n + j] = d;
            instance->cost[j * n + i] = d; 
        }
    }
}

/**
 * Check if the solution is feasible
 * @param solution array with the solution
 * @param cost cost of the solution
 * @param inst instance with the solution to be checked
 * @return 1 if the solution is feasible, 0 otherwise
 */
int check_sol(int* solution, double cost, instance* inst){
    
    int* count = (int*)calloc(inst->nnodes, sizeof(int));
    int n = inst->nnodes;

    // printf("Check solution: ");
    //     for (int i = 0; i < inst->nnodes + 1; i++) {
    //         printf("%d ", solution[i]);
    //     }
    //     printf("\n\n");

    for (int i = 0; i < inst->nnodes; i++){
        count[solution[i]]++;
    }

    for (int i = 0; i < n; i++){
        if (count[i] != 1){
            printf("Node %d appears %d times in the solution\n", i, count[i]);
            return 0;
        }
    }

    double total_cost = 0;
    for (int i = 0; i < n; i++){
        total_cost += inst->cost[solution[i] * n + solution[i+1]];
    }

    if (fabs(total_cost - cost) > EPS_COST){
        printf("Computed cost is different from the input cost\n");
        return 0;
    }

    return 1;
}

/**
 * Update the best solution found so far
 * @param inst instance with the solution
 * @param solution array with the solution
 * @param cost cost of the solution
 */
void update_best_sol(instance* inst, int* solution, double cost) {
    if (inst->best_cost > cost) {
        inst->best_cost = cost;

        for (int i = 0; i < inst->nnodes + 1; i++) {
            inst->best_sol[i] = solution[i];
        }

        if (!check_sol(inst->best_sol, inst->best_cost, inst)) {
            print_error("Invalid solution");
        }
    }
}

/**
 * Swap two elements in an array
 * @param arr array with the elements
 * @param i index of the first element
 * @param j index of the second element
 */
void swap(int* arr, int i, int j) {
    if (i != j) {
        int temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
    }
}

/**
 * Plot the solution using gnuplot and save the output as a PNG file
 * @param inst instance with the solution to be plotted
 */
void plot_solution(instance *inst, int* solution) {
    FILE *gnuplot = popen("gnuplot -persist", "w");
    if (gnuplot == NULL) { 
        printf("Error opening gnuplot\n");
        return;
    }

    // Set up the Gnuplot configuration
    fprintf(gnuplot, "set terminal pngcairo enhanced color font 'Helvetica,12'\n");  // Set PNG output
    fprintf(gnuplot, "set output '../data/solution.png'\n");
    fprintf(gnuplot, "set title 'TSP Solution' font 'Helvetica,16\n");

    // Define the style
    fprintf(gnuplot, "set style line 1 lc rgb '#FF0000' lt 1 lw 2 pt 7 ps 2\n");

    // Plot command
    fprintf(gnuplot, "plot '-' with linespoints ls 1 title 'TSP solution'\n");

    // Loop through the solution sequence
    for (int i = 0; i < (inst->nnodes + 1); i++) {
        int node = solution[i];  // Get the node index
        fprintf(gnuplot, "%lf %lf\n", inst->points[node].x, inst->points[node].y);
    }

    // Close the tour by adding the first node at the end
    fprintf(gnuplot, "%lf %lf\n", inst->points[solution[0]].x, inst->points[solution[0]].y);

    // End data input
    fprintf(gnuplot, "e\n");

    fflush(gnuplot);
    pclose(gnuplot);  // Close Gnuplot properly

    return;
}

/**
 * Apply the 2-opt refinement to the solution
 */
void refinement_two_opt(int* solution, instance* inst) {
    int* temp_solution = (int*)malloc((inst->nnodes + 1) * sizeof(int));
    memcpy(temp_solution, solution, (inst->nnodes + 1) * sizeof(int));
    double temp_cost = inst->best_cost;
    int n = inst->nnodes;

    int improvement = 1;

    while (improvement) {
        improvement = 0;
        double best_delta = 0;
        int best_i = -1;
        int best_j = -1;

        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 1; j < n; j++) {
                double delta = inst->cost[temp_solution[i] * n + temp_solution[j]] +
                               inst->cost[temp_solution[i + 1] * n + temp_solution[j + 1]] -
                               inst->cost[temp_solution[i] * n + temp_solution[i + 1]] -
                               inst->cost[temp_solution[j] * n + temp_solution[j + 1]];

                if (delta < best_delta) {
                    best_delta = delta;
                    best_i = i;
                    best_j = j;
                }
            }
        }
                
        if (best_delta < -EPS_COST) {
            // Perform the 2-opt swap
            int i = best_i+1;
            int j = best_j;
            while (i < j) {
                swap(temp_solution, i, j);
                i++;
                j--;
            }
            temp_cost += best_delta;
            improvement = 1;
        }

    }

    if (check_sol(temp_solution, temp_cost, inst)){
        update_best_sol(inst, temp_solution, temp_cost);
    }

    free(temp_solution);
}