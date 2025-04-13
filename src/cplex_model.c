#include <cplex_model.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cplex.h>  // Proper CPLEX header

//#define DEBUG    // da commentare se non si vuole il debugging
#define EPS 1e-5


/**
 * Build the solution from the output of cplex model
 * @param xstar
 * @param inst
 * @param succ
 * @param comp
 * @param ncomp
 */
void build_sol(const double *xstar, instance *inst, int *succ, int *comp, int *ncomp) {
	// Check for NULL pointers
	if (succ == NULL) {
		printf("succ NULL\n");
	}

	if (comp == NULL) {
		printf("comp NULL\n");
	}

	if (xstar == NULL) {
		printf("xstar NULL\n");
	}

	assert(succ != NULL);
	assert(comp != NULL);
	assert(xstar != NULL);

#ifdef DEBUG
	int *degree = calloc(inst->nnodes, sizeof(int));
	for ( int i = 0; i < inst->nnodes; i++ )
	{
		for ( int j = i+1; j < inst->nnodes; j++ )
		{
			int k = xpos(i,j,inst);
			if ( (fabs(xstar[k]) > EPS) && (fabs(xstar[k]-1.0) > EPS)) print_error(" wrong xstar in build_sol()");
			if ( xstar[k] > 0.5 ) 
			{
				++degree[i];
				++degree[j];
			}
		}
	}

	for ( int i = 0; i < inst->nnodes; i++ ) {
		if ( degree[i] != 2 ) {
			print_error("wrong degree in build_sol()");
		 }
	}	
	free(degree);
#endif

	*ncomp = 0;
	for ( int i = 0; i < inst->nnodes; i++ )
	{
		succ[i] = -1;
		comp[i] = -1;
	}
	
	for ( int start = 0; start < inst->nnodes; start++ )
	{
		if ( comp[start] >= 0 ) continue;  // node "start" was already visited, just skip it

		// a new component is found
		(*ncomp)++;
		int i = start;
		int done = 0;
		while ( !done )  // go and visit the current component
		{
			comp[i] = *ncomp;
			done = 1;
			for ( int j = 0; j < inst->nnodes; j++ )
			{
				if ( i != j && xstar[xpos(i,j,inst)] > 0.5 && comp[j] == -1 ) // the edge [i,j] is selected in xstar and j was not visited before 
				{
					succ[i] = j;
					i = j;
					done = 0;
					break;
				}
			}
		}	
		succ[i] = start;  // last arc to close the cycle
		
		// go to the next component...
	}
}

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
        file = fopen(filename, "a");
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
 * Maps the pair (i,j) to a single index in the cost matrix
 * @param i
 * @param j
 * @param inst
 * @return
 */
int xpos(int i, int j, instance *inst) {
    if (i == j) print_error("i == j in xpos");
    if (i > j) return xpos(j, i, inst);
    int pos = i * inst->nnodes + j - ((i + 1) * (i + 2)) / 2;
    return pos;
}

/**
 * Build cplex model for TSP problem
 * @param inst
 * @param env
 * @param lp
 */
void build_model(instance *inst, CPXENVptr env, CPXLPptr lp) {
    int izero = 0;
    char binary = 'B';

    char **cname = (char **)calloc(1, sizeof(char *));
    if (cname == NULL) {
        print_error("Memory allocation error for cname");
    }
	assert(cname != NULL);
    cname[0] = (char *)calloc(100, sizeof(char));
    if (cname[0] == NULL) {
        print_error("Memory allocation error for cname[0]");
    }

    // Add binary variables x(i,j) for i < j
    for (int i = 0; i < inst->nnodes; i++) {
        for (int j = i+1; j < inst->nnodes; j++) {
            sprintf(cname[0], "x(%d,%d)", i+1, j+1);
        	double obj = inst->cost_matrix[i * inst->nnodes + j]; // cost == distance
            double lb = 0.0;
            double ub = 1.0;
            if (CPXnewcols(env, lp, 1, &obj, &lb, &ub, &binary, cname))
                print_error("wrong CPXnewcols on x var.s");
            if (CPXgetnumcols(env, lp)-1 != xpos(i, j, inst))
                print_error("wrong position for x var.s");
        }
    }

    // Add degree constraints
    int *index = (int *)malloc(inst->nnodes * sizeof(int));
    double *value = (double *)malloc(inst->nnodes * sizeof(double));

    for (int h = 0; h < inst->nnodes; h++) {
        double rhs = 2.0;
        char sense = 'E';                     // 'E' for equality constraint
        sprintf(cname[0], "degree(%d)", h+1);
        int nnz = 0;
        for (int i = 0; i < inst->nnodes; i++) {
            if (i == h) continue;
            index[nnz] = xpos(i, h, inst);
            value[nnz] = 1.0;
            nnz++;
        }

        if (CPXaddrows(env, lp, 0, 1, nnz, &rhs, &sense, &izero, index, value, NULL, cname))
            print_error("wrong CPXaddrows [degree]");
    }

    // Add subtour elimination constraints (SEC)
    // This is a critical addition for the TSP model
    // We'll use lazy constraints approach
    if (CPXsetintparam(env, CPXPARAM_MIP_Strategy_CallbackReducedLP, 1))
        print_error("CPXsetintparam() error for lazy constraints");

    free(value);
    free(index);

    // Write model to file for debugging
    CPXwriteprob(env, lp, "../data/model.lp", NULL);

    free(cname[0]);
    free(cname);
}

/**
 * Helper function to add subtour elimination constraints. This would be used in a callback, but we're simplifying for now
 * @param env
 * @param lp
 * @param comp_index
 * @param comp
 * @param inst
 */
void add_sec(CPXENVptr env, CPXLPptr lp, int comp_index, int *comp, instance *inst) {
    if (comp == NULL) {
        print_error("comp array is NULL");
    }
	assert(comp != NULL);

    int izero = 0;
    char sense = 'L';
    double rhs = - 1.0;
    int nnz = 0;

    int ncols = CPXgetnumcols(env, lp);
    int *index = (int *)malloc(ncols * sizeof(int));
    double *value = (double *)malloc(ncols * sizeof(double));

    char **cname = (char **)calloc(1, sizeof(char *));
    if (cname == NULL) {
        print_error("Memory allocation error for cname");
    }
	assert(cname != NULL);

    cname[0] = (char *)calloc(100, sizeof(char));
    if (cname[0] == NULL) {
        print_error("Memory allocation error for cname[0]");
    }

    for (int i = 0; i < inst->nnodes; i++) {
        if (comp[i] != comp_index) { continue; }
        rhs += 1.0;
        for (int j = i+1; j < inst->nnodes; j++) {
            if (j == i) { continue; }
            if (comp[j] != comp_index) { continue; }
            index[nnz] = xpos(i, j, inst);
            value[nnz] = 1.0;
            nnz++;
            sprintf(cname[0], "SEC(%d)", comp_index);
        }
    }
    if (CPXaddrows(env, lp, 0, 1, nnz, &rhs, &sense, &izero, index, value, NULL, cname))
            print_error("wrong CPXaddrows [degree]");
    
    free(cname[0]);
    free(cname);
    free(value);
    free(index);
}

/**
 * Solve the TSP problem using CPLEX
 * @param inst
 * @return
 */
int TSPopt(instance *inst) {
	/************************************************************************************************
	 * Maybe a tour struct should be passed as input to store the final solution of CPLEX
	 ***********************************************************************************************/
    // Open CPLEX model
    int error;
    CPXENVptr env = CPXopenCPLEX(&error);
    if (env == NULL) {
        print_error("CPXopenCPLEX() error");
    }
    CPXLPptr lp = CPXcreateprob(env, &error, "TSP");
    if (lp == NULL) {
        print_error("CPXcreateprob() error");
    }

    build_model(inst, env, lp);
	printf("Time limit: %lf\n", inst->timelimit);
    // Set CPLEX parameters
    if (CPXsetintparam(env, CPX_PARAM_SCRIND, 1)) // Enable screen output
        print_error("CPXsetintparam() error");
    if (CPXsetdblparam(env, CPX_PARAM_TILIM, inst->timelimit)) // Time limit
        print_error("CPXsetdblparam() error");
    if (CPXsetintparam(env, CPX_PARAM_MIPDISPLAY, 2)) // Verbosity level
        print_error("CPXsetintparam() error");
	if (CPXsetdblparam(env, CPX_PARAM_EPGAP, 1e-7)) // Set the optimality gap
		print_error("CPXsetdblparam() error");
	if (CPXsetintparam(env, CPXPARAM_RandomSeed, inst->seed))
		print_error("CPXsetintparam() error");

	int ncols = CPXgetnumcols(env, lp);
	double objval = 0.0;
	int iter = 0;
	int ncomp = 9999;
	double *xstar = (double *)calloc(ncols, sizeof(double));
	int *succ = (int *)malloc(inst->nnodes * sizeof(int));
    int *comp = (int *)malloc(inst->nnodes * sizeof(int));
	// Flag to indicate if 2-opt is applied. Will be set to one if
	// CPLEX is not able to solve the problem to the optimality
	int two_opt_flag = 0;

	while (ncomp >= 2) {
        iter++;
		if (second() - inst->tstart > inst->timelimit) {
			printf("Time limit reached\n");
			break;
		}
		// apply to cplex the residual time left to solve the problem
		double residual_time = inst->timelimit - (second() - inst->tstart); // The residual time limit that is left
		CPXsetdblparam(env, CPX_PARAM_TILIM, residual_time);

		if ( CPXmipopt(env, lp) ) { print_error("CPXmipopt() error"); }
		if ( CPXgetx(env, lp, xstar, 0, ncols-1) ) { print_error("CPXgetx() error"); }
		build_sol(xstar, inst, succ, comp, &ncomp);
        if ( CPXgetobjval(env, lp, &objval) ) { print_error("CPXgetobjval() error"); }
		if (VERBOSE > 1000) {
			printf("Iter %4d, Lower Bound: %10.2lf, Number of components: %4d, Time: %5.2lfs\n", iter, objval, ncomp, second()-inst->tstart);
			fflush(NULL);
		}

		if (ncomp >= 2) {
			for (int k = 1; k <= ncomp; k++) {
				add_sec(env, lp, k, comp, inst);
			}
		}
        save_history_benders(objval, second()-inst->tstart, "../data/history_benders.txt");
        fflush(NULL);
	}

	if (ncomp >= 2) {
		if (VERBOSE > 1000 ) { printf("Number of components before solving: %d\n", ncomp); }
		patching_heuristic(succ, &ncomp, comp, inst);
		two_opt_flag = 1;
		if (VERBOSE > 1000 ) { printf("Number of components after patching: %d\n", ncomp); }
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

	tour* solution = (tour *)malloc(sizeof(tour));
	solution->cost = INF_COST;
	solution->path = (int *)malloc((inst->nnodes + 1) * sizeof(int));
	//fill solution->path with the solution from succ array starting from node 0
	int start = 0;
	int current = start;
	do {
		solution->path[current] = succ[current];
		current = succ[current];
	} while (current != start);
	solution->path[inst->nnodes] = solution->path[0]; // Close the tour

	// Compute solution cost and check its feasibility
	compute_solution_cost(solution, inst);
	if ( VERBOSE > 1000 ) { printf("Cost of the solution before 2-Opt: %lf\n", solution->cost); }
	if (two_opt_flag) {
		// Reset the time limit to allow for 2-Opt
		inst->tstart = second();
		two_opt(solution, inst);
	}

	if ( VERBOSE > 1000 ) { printf("Cost of the solution after 2-Opt: %lf\n", solution->cost); }
	check_sol(solution->path, solution->cost, inst);
	if ( VERBOSE > 1000 ) { printf("Cost of the solution: %lf\n", solution->cost); }
	plot_solution(inst, solution->path);

    plot_succ_path(succ, inst, "../data/solution_cplex.png");
    plot_cost_benders("../data/history_benders.txt", "../data/history_benders.png");

	// Free allocated memory
	free(solution->path);
	free(solution);
	free(comp);
    free(succ);
    free(xstar);

    // Free and close CPLEX model
    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);

    return 0;
}

void patching_heuristic(int* succ, int* ncomp, int* comp, instance* inst) {
	if (succ == NULL) { print_error("Error occurred while allocating memory for succ\n"); }
	if (ncomp == NULL) { print_error("Error occurred while allocating memory for ncomp\n"); }
	if (comp == NULL) { print_error("Error occurred while allocating memory for comp\n"); }
	if (inst == NULL) { print_error("Error occurred while allocating memory for inst\n"); }
	assert(ncomp > 0);
	assert(succ != NULL);
	assert(comp != NULL);
	assert(inst != NULL);
	assert(ncomp != NULL);

	while (*ncomp > 1) {
		int best_i = -1;
		int best_j = -1;
		int cross_flag = -1;
		double best_delta = INF_COST;
		for (int i = 0; i < inst->nnodes; i++) {
			for (int j = 0; j < inst->nnodes; j++) {
				if (i == j) continue;

				if (comp[i] != comp[j]) {
					// best delta between cross swap or straight swap
					double delta_straight = inst->cost_matrix[i * inst->nnodes + j] +
						inst->cost_matrix[succ[i] * inst->nnodes + succ[j]] -
					 	inst->cost_matrix[i * inst->nnodes + succ[i]] -
					 	inst->cost_matrix[j * inst->nnodes + succ[j]];
                    //double delta_straight = INF_COST;

					double delta_cross = inst->cost_matrix[i * inst->nnodes + succ[j]] +
						inst->cost_matrix[succ[i] * inst->nnodes + j] -
						inst->cost_matrix[i * inst->nnodes + succ[i]] -
						inst->cost_matrix[j * inst->nnodes + succ[j]];
					
                    if (delta_straight + EPS_COST < best_delta || delta_cross + EPS_COST < best_delta) {
						if (delta_cross < delta_straight) {
							best_delta = delta_cross;
							best_i = i;
							best_j = j;
							cross_flag = 1;
						} else {
							best_delta = delta_straight;
							best_i = i;
							best_j = j;
							cross_flag = 0;
						}
					}
				}
			}
		}
		if (cross_flag) {
			int temp = succ[best_i];
			succ[best_i] = succ[best_j];
			succ[best_j] = temp;
			for (int k = 0; k < inst->nnodes; k++) {
				if (comp[k] == comp[best_j] && k != best_j) {
					comp[k] = comp[best_i];
				}
			}
            comp[best_j] = comp[best_i];
			(*ncomp)--;
		} else {
			int temp = succ[best_i];
			succ[best_i] = best_j;
			int past_node = succ[best_j];
			int current_node = succ[past_node];
			succ[past_node] = temp;

			// reverse the component of best_j and succ[best_j]
			// Start from succ[succ[best_j]] and reverse until best_j
			int next_node = succ[current_node];
			while (next_node != best_j) {
				succ[current_node] = past_node;
				past_node = current_node;
				current_node = next_node;
				next_node = succ[current_node];
			}
			succ[current_node] = past_node;
			succ[best_j] = current_node;

			for (int k = 0; k < inst->nnodes; k++) {
				if (comp[k] == comp[best_j] && k != best_j) {
					comp[k] = comp[best_i];
				}
			}
			comp[best_j] = comp[best_i];
			(*ncomp)--;
		}
	}
	if (VERBOSE > 2000) { printf("Finished patching heuristic\n"); }
}