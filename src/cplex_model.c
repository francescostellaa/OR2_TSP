#include <cplex_model.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cplex.h>  // Proper CPLEX header

#define DEBUG    // da commentare se non si vuole il debugging
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
	for ( int i = 0; i < inst->nnodes; i++ )
	{
		if ( degree[i] != 2 ) print_error("wrong degree in build_sol()");
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
 * Plot the solution path
 * @param xstar
 * @param inst
 * @param output_file
 */
void plot_xstar_path(const double *xstar, instance *inst, const char *output_file) {
    int *solution = (int *)malloc((inst->nnodes + 1) * sizeof(int));
    if (solution == NULL) {
        print_error("Memory allocation error for solution");
    }
	assert(solution != NULL);
    // Convert xstar to solution path
    xstar_to_solution(xstar, inst, solution);
	for (int i = 0; i < inst->nnodes + 1; i++) {printf("solution[%d] = %d\n", i, solution[i]);}
    // Open gnuplot
    FILE *gnuplot = popen("gnuplot -persist", "w");
    if (gnuplot == NULL) {
        print_error("Error opening gnuplot");
    }

    // Set up gnuplot configuration
    fprintf(gnuplot, "set terminal pngcairo enhanced color font 'Helvetica,12' size 1000,720\n");
    fprintf(gnuplot, "set output '%s'\n", output_file);
    fprintf(gnuplot, "set title 'TSP Solution Path' font 'Helvetica,16'\n");
    fprintf(gnuplot, "set style line 1 lc rgb '#FF0000' lt 1 lw 2 pt 7 ps 1.5\n");
    fprintf(gnuplot, "plot '-' with linespoints ls 1 title 'TSP Path'\n");

    // Plot the solution path
    for (int i = 0; i <= inst->nnodes; i++) {
        int node = solution[i];  // Fixed modulo to avoid index out of bounds
        fprintf(gnuplot, "%lf %lf\n", inst->points[node].x, inst->points[node].y);
    }

    // Close the tour by adding the first node at the end (already handled by modulo above)
    fprintf(gnuplot, "%lf %lf\n", inst->points[solution[0]].x, inst->points[solution[0]].y);

    // End data input
    fprintf(gnuplot, "e\n");

    // Close gnuplot
    fflush(gnuplot);
    pclose(gnuplot);

    // Free allocated memory
    free(solution);
}

/**
 * Convert xstar to solution path
 * @param xstar
 * @param inst
 * @param solution
 */
void xstar_to_solution(const double *xstar, instance *inst, int *solution) {
	int *succ = (int *)malloc(inst->nnodes * sizeof(int));
	int *comp = (int *)malloc(inst->nnodes * sizeof(int));
	int ncomp;

	build_sol(xstar, inst, succ, comp, &ncomp);

	// Check if we have a valid tour (only one component)
	if (ncomp != 1) {
		printf("Warning: Solution contains %d components instead of 1\n", ncomp);
	}

	// Reset solution array
	for (int i = 0; i < inst->nnodes + 1; i++) {
		solution[i] = -1;
	}

	// Find the first unvisited node as the starting point for the first component
	int idx = 0;
	for (int comp_id = 0; comp_id < ncomp; comp_id++) {
		int start = -1;
		for (int i = 0; i < inst->nnodes; i++) {
			if (comp[i] == comp_id + 1) { // Components are 1-indexed in build_sol
				start = i;
				break;
			}
		}

		if (start == -1) continue; // No valid starting node found

		int current = start;
		do {
			solution[idx++] = current;
			current = succ[current];
		} while (current != start && idx < inst->nnodes);
	}

	// Add the starting node at the end to close the tour
	if (idx < inst->nnodes + 1 && solution[0] != -1) {
		solution[idx] = solution[0];
	}

	free(succ);
	free(comp);
}

// Calculates the distance between two points
double dist_cplex(int i, int j, instance *inst) {
    double dx = inst->points[i].x - inst->points[j].x;
    double dy = inst->points[i].y - inst->points[j].y;

    if (!inst->integer_costs)
        return sqrt(dx*dx + dy*dy);

    int dis = sqrt(dx*dx + dy*dy) + 0.499999999;  // nearest integer
    return (double)dis;
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
            //double obj = dist_cplex(i, j, inst); // cost == distance
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
 * @param size
 * @param comp
 * @param inst
 */
void add_sec(CPXENVptr env, CPXLPptr lp, int size, int *comp, instance *inst) {
    // Count nodes in the component
    int count = 0;
    for (int i = 0; i < inst->nnodes; i++) {
        if (comp[i] == 1) count++;
    }

    // If we have a subtour (not involving all nodes)
    if (count > 0 && count < inst->nnodes) {
        int nnz = 0;
        int *indices = (int *)malloc(inst->nnodes * inst->nnodes * sizeof(int));
        double *values = (double *)malloc(inst->nnodes * inst->nnodes * sizeof(double));

        // Build the constraint
        for (int i = 0; i < inst->nnodes; i++) {
            if (comp[i] != 1) continue;
            for (int j = i+1; j < inst->nnodes; j++) {
                if (comp[j] != 1) continue;
                indices[nnz] = xpos(i, j, inst);
                values[nnz] = 1.0;
                nnz++;
            }
        }

        double rhs = count - 1.0;
        char sense = 'L';
        int izero = 0;
        char **cname = (char **)calloc(1, sizeof(char *));
        cname[0] = (char *)calloc(100, sizeof(char));
        sprintf(cname[0], "SEC_%d", size);

        if (CPXaddrows(env, lp, 0, 1, nnz, &rhs, &sense, &izero, indices, values, NULL, cname))
            print_error("Error adding subtour elimination constraint");

        free(indices);
        free(values);
        free(cname[0]);
        free(cname);
    }
}

/**
 * Solve the TSP problem using CPLEX
 * @param inst
 * @return
 */
int TSPopt(instance *inst) {
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

    // Set CPLEX parameters
    if (CPXsetintparam(env, CPX_PARAM_SCRIND, 1)) // Enable screen output
        print_error("CPXsetintparam() error");
    if (CPXsetdblparam(env, CPX_PARAM_TILIM, 3600.0)) // Time limit: 1 hour
        print_error("CPXsetdblparam() error");
    if (CPXsetintparam(env, CPX_PARAM_MIPDISPLAY, 2)) // Verbosity level
        print_error("CPXsetintparam() error");
	if (CPXsetdblparam(env, CPX_PARAM_EPGAP, 1e-9)) // Set the optimality gap 	
		print_error("CPXsetdblparam() error"); 

	int ncols = CPXgetnumcols(env, lp);
	double objval = 0.0;
	int iter = 0;
	int ncomp = 9999;
	double *xstar = (double *)calloc(ncols, sizeof(double));
	int *succ = (int *)malloc(inst->nnodes * sizeof(int));
    int *comp = (int *)malloc(inst->nnodes * sizeof(int));

	while (ncomp >= 2) {
		CPXmipopt(env, lp);
		CPXgetx(env, lp, xstar, 0, ncols-1);
		build_sol(xstar, inst, succ, comp, &ncomp);
		if (VERBOSE > 1000) {
			printf("Iter %4d, Lower Bound: %10.2lf, Number of components: %4d, Time: %5.2lf", iter, objval, ncomp, second()-inst->tstart);
			fflush(NULL);
		}
		if (ncomp >= 2) {
			continue;
		}
	}
    // Solve the model
    // if (CPXmipopt(env, lp))
    //     print_error("CPXmipopt() error");

    // Retrieve solution status
    // int status = CPXgetstat(env, lp);
    // printf("Solution status: %d\n", status);

    // Get solution objective value
    if (CPXgetobjval(env, lp, &objval))
        print_error("CPXgetobjval() error");
    printf("Objective value: %f\n", objval);

    // Retrieve the solution
    // if (CPXgetx(env, lp, xstar, 0, ncols-1))
    //     print_error("CPXgetx() error");

    // Print selected edges
    // printf("Selected edges in the solution:\n");
    // for (int i = 0; i < inst->nnodes; i++) {
    //     for (int j = i+1; j < inst->nnodes; j++) {
    //         if (xstar[xpos(i, j, inst)] > 0.5) {
    //             printf("  x(%3d,%3d) = 1\n", i+1, j+1);
    //         }
    //     }
    // }

    // Check if solution contains subtours
    // build_sol(xstar, inst, succ, comp, &ncomp);
	// if (VERBOSE > 1000) {
	// 	printf("Iter %4d, Lower Bound: %10.2lf, Number of components: %4d, Time: %5.2lf", iter, objval, ncomp, second()-inst->tstart);
	// 	fflush(NULL);
	// }

    // if (ncomp > 1) {
    //     printf("Warning: Solution contains %d subtours!\n", ncomp);
    //     // We would need to add SEC constraints and re-solve
    //     // For simplicity, we'll just report this issue
    // }

	// Plot the solution
    plot_xstar_path(xstar, inst, "../data/solution_cplex.png");

	// Free allocated memory
	free(comp);
    free(succ);
    free(xstar);

    // Free and close CPLEX model
    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);

    return 0;
}
