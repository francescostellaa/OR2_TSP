#include <cplex_model.h>
#include <cplex_utilities.h>

//#define DEBUG    // da commentare se non si vuole il debugging

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
    int error = 0;
    CPXENVptr env = CPXopenCPLEX(&error);
    if (env == NULL) {
        char errormsg[CPXMESSAGEBUFSIZE];
		CPXgeterrorstring(env, error, errormsg);
        print_error(errormsg);
    }
    CPXLPptr lp = CPXcreateprob(env, &error, "TSP");
    if (lp == NULL) {
		char errormsg[CPXMESSAGEBUFSIZE];
		CPXgeterrorstring(env, error, errormsg);
        print_error(errormsg);
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
	double objval = INF_COST;
	int iter = 0;
	int ncomp = 9999;
	double *xstar = (double *)calloc(ncols, sizeof(double));
	int *succ = (int *)malloc(inst->nnodes * sizeof(int));
    int *comp = (int *)malloc(inst->nnodes * sizeof(int));
	// Flag to indicate if 2-opt is applied. Will be set to 1 if
	// CPLEX is not able to solve the problem to the optimality
	int two_opt_flag = 0;
	tour* solution = (tour *)malloc(sizeof(tour));

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
		int solstat = CPXgetstat(env, lp);
		if (solstat != CPXMIP_OPTIMAL && solstat != CPXMIP_OPTIMAL_TOL && solstat != CPXMIP_FEASIBLE) {
			printf("CPLEX Error: No feasible solution found for the given time limit.\nStatus: %d\n", solstat);
			exit(1); // Exit the loop or handle the error
		}
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
		if (ncomp >= 2) {
			patching_heuristic(succ, ncomp, comp, inst);
			reconstruct_sol(solution, succ, inst);
			two_opt(solution, inst);
			check_sol(solution->path, solution->cost, inst);
			if (VERBOSE > 100) { printf("Solution cost after 2-Opt: %lf\n", solution->cost); }
		}

        save_history_benders(objval, second()-inst->tstart, "../data/history_benders.txt");
        fflush(NULL);
	}

	if (iter <= 1) {
		print_error("Increase the time limit");
	}
	
	#ifdef DEBUG
		check_degrees(inst, xstar);	
	#endif

	reconstruct_sol(solution, succ, inst);
	plot_solution(inst, solution->path);
	update_best_sol(inst, solution);

	// Plot the solution path of both CPLEX
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


void patching_heuristic(int* succ, int ncomp, int* comp, instance* inst) {
	if (succ == NULL) { print_error("Error occurred while allocating memory for succ\n"); }
	if (comp == NULL) { print_error("Error occurred while allocating memory for comp\n"); }
	if (inst == NULL) { print_error("Error occurred while allocating memory for inst\n"); }

	while (ncomp > 1) {
		int best_i = -1;
		int best_j = -1;
		int cross_flag = -1;
		double best_delta = INF_COST;
		for (int i = 0; i < inst->nnodes; i++) {
			for (int j = 0; j < inst->nnodes; j++) {
				if (i == j) continue;

				if (comp[i] != comp[j]) {
					update_best_delta(i, j, succ, inst, &best_i, &best_j, &best_delta, &cross_flag);
				}
			}
		}
		if (cross_flag) {
			patch_cross_case(succ, comp, &ncomp, best_i, best_j, inst);
		} else {
			patch_straight_case(succ, comp, &ncomp, best_i, best_j, inst);
		}
	}
	if (VERBOSE > 2000) { printf("Finished patching heuristic\n"); }
}