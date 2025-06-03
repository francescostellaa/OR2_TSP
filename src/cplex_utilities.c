#include <cplex_utilities.h>
#include <cplex_model.h>

/**
 * Save the history of the Benders algorithm
 * @param cost
 * @param time
 * @param filename
 */
void save_history_benders(double cost, double time, const char* filename) {
    static int first_call = 0;

	FILE *file = NULL;
    // First call: erase file contents
    if (!first_call) {

        file = fopen(filename, "w");
        first_call = 1;
    } else {
        file = fopen(filename, "ab+");
    }

    if (file == NULL) {
        perror("Error opening file");
        return;
    }

    // Write current time and cost to file
    fprintf(file, "%.6f %.6f\n", time, cost);
    fclose(file);
}

/**
 * Plot the cost history of the Benders algorithm
 * @param input_file
 * @param output_file
 */
void plot_cost_benders(const char *input_file, const char *output_file) {
    FILE *gnuplot = popen("gnuplot -persist", "w");
    if (gnuplot == NULL) {
        perror("Error opening gnuplot");
        return;
    }

    // Set up GNUplot configuration
    fprintf(gnuplot, "set terminal pngcairo enhanced color font 'Helvetica,12' size 1000,720\n");
    fprintf(gnuplot, "set output '%s'\n", output_file);
    fprintf(gnuplot, "set title 'Benders Algorithm History' font 'Helvetica,16'\n");
    fprintf(gnuplot, "set xlabel 'Time (s)' font 'Helvetica,12'\n");
    fprintf(gnuplot, "set ylabel 'Cost' font 'Helvetica,12'\n");
    fprintf(gnuplot, "set grid\n");
    fprintf(gnuplot, "plot '%s' using 1:2 with linespoints title 'Cost' lw 2 lc rgb '#FF0000' pt 7 ps 1.5\n", input_file);

    // Close GNUplot
    fflush(gnuplot);
    pclose(gnuplot);
}

/**
 * Plot the solution path based on the succ array
 * @param succ
 * @param inst
 * @param output_file
 */
void plot_succ_path(const int *succ, instance *inst, const char *output_file) {
    if (succ == NULL) {
        print_error("succ array is NULL");
    }
	assert(succ != NULL);
    // Open gnuplot
    FILE *gnuplot = popen("gnuplot -persist", "w");
    if (gnuplot == NULL) {
        print_error("Error opening gnuplot");
    }

    // Set up gnuplot configuration
    fprintf(gnuplot, "set terminal pngcairo enhanced color font 'Helvetica,12' size 1000,720\n");
    fprintf(gnuplot, "set output '%s'\n", output_file);
    fprintf(gnuplot, "set title 'TSP Solution Path (Based on succ)' font 'Helvetica,16'\n");
    fprintf(gnuplot, "set style line 1 lc rgb '#FF0000' lt 1 lw 2 pt 7 ps 1.5\n");
    fprintf(gnuplot, "plot '-' with linespoints ls 1 title 'TSP Path'\n");

    // Plot the solution path based on the succ array
    int start = 0; // Assuming the tour starts at node 0
    int current = start;

    do {
        fprintf(gnuplot, "%lf %lf\n", inst->points[current].x, inst->points[current].y);
        current = succ[current];
    } while (current != start);

    // Close the tour by adding the first node at the end
    fprintf(gnuplot, "%lf %lf\n", inst->points[start].x, inst->points[start].y);

    // End data input
    fprintf(gnuplot, "e\n");

    // Close gnuplot
    fflush(gnuplot);
    pclose(gnuplot);
}

/**
 * Convert a solution obtained from a heuristic to a successor array
 * @param succ
 * @param solution
 * @param inst
 */
void from_solution_to_succ(int* succ, tour* solution, const instance* inst) {

	if (solution == NULL) {
		print_error("solution is NULL");
	}
	if (succ == NULL) {
		print_error("succ is NULL");
	}
	if (inst == NULL) {
		print_error("inst is NULL");
	}

	//fill succ with the solution obtained using heuristic
	int start = 0;
	int current = start;
	for (int i = 0; i < inst->nnodes; i++) {
		succ[solution->path[i]] = solution->path[i + 1];
    }

}

/**
 * Reconstruct the solution from the successors array
 * @param solution
 * @param succ
 * @param inst
 */
void reconstruct_sol(tour* solution, int* succ, const instance* inst) {
	if (solution == NULL) print_error("solution is NULL");
	if (succ == NULL) print_error("succ is NULL");
	if (inst == NULL) print_error("inst is NULL");

	solution->cost = INF_COST;
	solution->path = (int *)malloc((inst->nnodes + 1) * sizeof(int));
	if (solution->path == NULL) print_error("Memory allocation failed for solution path");

	//fill solution->path with the solution from succ array starting from node 0
	int start = 0;
	int current = start;

	for (int i = 0; i < inst->nnodes; i++) {
		solution->path[i] = current;
		current = succ[current];
	}
	solution->path[inst->nnodes] = start; // close the tour
	compute_solution_cost(solution, inst);
}

/**
 * Check the degrees of the nodes in the solution
 * @param inst
 * @param xstar
 */
void check_degrees(instance* inst, const double* xstar) {
	if (inst == NULL) {
		print_error("inst is NULL");
	}
	if (xstar == NULL) {
		print_error("xstar is NULL");
	}
	int *degree = calloc(inst->nnodes, sizeof(int));
	for ( int i = 0; i < inst->nnodes; i++ ) {
		for ( int j = i+1; j < inst->nnodes; j++ ) {
			int k = xpos(i,j,inst);
			if ( (fabs(xstar[k]) > EPS) && (fabs(xstar[k]-1.0) > EPS)) print_error(" wrong xstar in build_sol()");
			if ( xstar[k] > 0.5 ) {
				++degree[i];
				++degree[j];
			}
		}
	}

	// Checks if the degree of each node is 2
	for ( int i = 0; i < inst->nnodes; i++ ) {
		if ( degree[i] != 2 ) {
			print_error("wrong degree in build_sol()");
		 }
	}	
	free(degree);
}

/**
 * Compute the delta cost for a straight patching operation
 * @param i First node index
 * @param j Second node index
 * @param succ Successor array
 * @param inst Instance containing the cost matrix
 * @return The computed delta cost for the straight patching operation
 */
double compute_delta_straight(int i, int j, int* succ, instance* inst) {

	double delta_straight = inst->cost_matrix[i * inst->nnodes + j] +
		inst->cost_matrix[succ[i] * inst->nnodes + succ[j]] -
		inst->cost_matrix[i * inst->nnodes + succ[i]] -
		inst->cost_matrix[j * inst->nnodes + succ[j]];
	return delta_straight;
}

/**
 * Compute the delta cost for a cross patching operation
 * @param i First node index
 * @param j Second node index
 * @param succ Successor array
 * @param inst Instance containing the cost matrix
 * @return The computed delta cost for the cross patching operation
 */
double compute_delta_cross(int i, int j, int* succ, instance* inst) {

	double delta_cross = inst->cost_matrix[i * inst->nnodes + succ[j]] +
		inst->cost_matrix[succ[i] * inst->nnodes + j] -
		inst->cost_matrix[i * inst->nnodes + succ[i]] -
		inst->cost_matrix[j * inst->nnodes + succ[j]];
	return delta_cross;
}

/**
 * Reverse the component of a cycle in the straight patching case.
 * @param succ Successor array.
 * @param best_j Best j index.
 * @param current_node Current node being processed.
 * @param past_node Previous node in the cycle.
 */
void reverse_component(int *succ, int best_j, int current_node, int past_node) {
    int next_node = succ[current_node];
    while (next_node != best_j) {
        succ[current_node] = past_node;
        past_node = current_node;
        current_node = next_node;
        next_node = succ[current_node];
    }
    succ[current_node] = past_node;
    succ[best_j] = current_node;
}
