#include <cplex_model.h>
#include <cplex_utilities.h>

//#define DEBUG    // da commentare se non si vuole il debugging

#include <stdio.h>
#include <termios.h>
#include <unistd.h>

/**
 * Build the solution from the output of cplex model
 * @param xstar solution vector from cplex
 * @param inst instance containing the problem data
 * @param succ successor array to be filled
 * @param comp component array to be filled
 * @param ncomp number of components found in the solution
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
 * @param inst instance containing the problem data
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
 * @param inst instance containing the problem data
 * @param env CPLEX environment pointer
 * @param lp CPLEX LP pointer
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
 * Add a subtour elimination constraint (SEC) to the model
 * @param nnz number of non-zero entries in the constraint
 * @param rhs right-hand side value of the constraint
 * @param comp_index index of the component for which the SEC is being added
 * @param index array to hold the indices of the variables in the constraint
 * @param value array to hold the coefficients of the variables in the constraint
 * @param cname array to hold the name of the constraint
 * @param comp component array indicating the component each node belongs to
 * @param inst instance containing the problem data
 */
void add_sec(int* nnz, double* rhs, int comp_index, int* index, double* value, char** cname, int *comp, instance *inst) {
    if (comp == NULL) {print_error("comp array is NULL");}
    if (cname == NULL) {print_error("Memory allocation error for cname");}
	if (cname[0] == NULL) {
		print_error("Memory allocation error for cname[0]");
	}
    for (int i = 0; i < inst->nnodes; i++) {
        if (comp[i] != comp_index) { continue; }
        *rhs += 1.0;
        for (int j = i+1; j < inst->nnodes; j++) {
            if (j == i) { continue; }
            if (comp[j] != comp_index) { continue; }
            index[*nnz] = xpos(i, j, inst);
            value[*nnz] = 1.0;
            (*nnz)++;
            sprintf(cname[0], "SEC(%d)", comp_index);
        }
    }
}

/**
 * Solve the model using CPLEX
 * @param env CPLEX environment pointer
 * @param lp CPLEX LP pointer
 * @param inst instance containing the problem data
 * @param xstar solution vector to be filled
 * @param succ successor array to be filled
 * @param comp component array to be filled
 * @param ncomp number of components found in the solution
 * @param objval pointer to store the objective value of the solution
 */
void solver(CPXENVptr env, CPXLPptr lp, instance *inst, double *xstar, int *succ, int *comp, int *ncomp, double *objval) {

	if (inst->mode == 1 || inst->mode == 4 || inst->mode == 5 || (inst->mode == 6)) {
		warm_start(env, lp, succ, inst);
	}

	if (CPXmipopt(env, lp)) {
        print_error("CPXmipopt() error");
    }
	// Retrive CPLEX solution
    if (CPXgetx(env, lp, xstar, 0, inst->ncols - 1)) {
        print_error("CPXgetx() error");
    }
	// Build solution from xstar
    build_sol(xstar, inst, succ, comp, ncomp);

    if (CPXgetobjval(env, lp, objval)) {
        print_error("CPXgetobjval() error");
    }

}


/**
 * Warm start the solution
 * @param succ successor array to be filled with the initial solution
 * @param inst instance containing the problem data
 * @return 0 on success, non-zero on failure
 */
int warm_start(CPXENVptr env, CPXLPptr lp, int* succ, instance *inst) {
	tour* solution = malloc(sizeof(tour));
	solution->path = malloc((inst->nnodes + 1) * sizeof(int));
	solution->cost = 0.0;

	if (greedy(0, solution, 1, inst)) { print_error("Error during heuristic\n");}
	if (VERBOSE > 1000) {
		printf("Initial solution cost: %lf\n found after %5.2lf second\n", solution->cost, second() - inst->tstart);
		fflush(NULL);
	}
	// Build the successors array from the solution to suit CPLEX requirements
	from_solution_to_succ(succ, solution, inst);
	double *xheu = calloc(inst->ncols, sizeof(double));  // all zeros, initially
	for ( int i = 0; i < inst->nnodes; i++ ) xheu[xpos(i,succ[i],inst)] = 1.0;
	int *ind = malloc(inst->ncols * sizeof(int));
	for ( int j = 0; j < inst->ncols; j++ ) ind[j] = j;

	int effortlevel = CPX_MIPSTART_NOCHECK;
	int beg = 0;
	// Set the mipstart solution in CPLEX
	if ( CPXaddmipstarts(env, lp, 1, inst->ncols, &beg, ind, xheu, &effortlevel, NULL)) {
		print_error("CPXaddmipstarts() error");
	}

	// free allocated memory
	free(ind);
	free(xheu);
	free(solution->path);
	free(solution);
	return 0;
}

/**
 * Berder's loop algorithm
 * @param env CPLEX environment pointer
 * @param lp CPLEX LP pointer
 * @param inst instance containing the problem data
 * @param succ successor array to be filled
 * @param ncomp number of components found in the solution
 * @param solution tour structure to store the solution
 * @return 0 on success, non-zero on failure
 */
int benders(CPXENVptr env, CPXLPptr lp, instance *inst, int *succ, int ncomp, tour* solution) {

	int iter = 0;
	double objval = INF_COST;

	char sense = 'L';
	int izero = 0;

	double *xstar = (double *)calloc(inst->ncols, sizeof(double));
	int *comp = (int *)malloc(inst->nnodes * sizeof(int));

	while (ncomp >= 2) {
        iter++;
		if (second() - inst->tstart > inst->timelimit) {
			printf("Time limit reached\n");
			break;
		}
		// apply to cplex the residual time left to solve the problem
		double residual_time = inst->timelimit - (second() - inst->tstart); // The residual time limit that is left
		CPXsetdblparam(env, CPX_PARAM_TILIM, residual_time);
		// Solve the model
		solver(env, lp, inst, xstar, succ, comp, &ncomp, &objval);
		
		if (VERBOSE > 1000) {
			printf("Iter %4d, Lower Bound: %10.2lf, Number of components: %4d, Time: %5.2lfs\n", iter, objval, ncomp, second()-inst->tstart);
			fflush(NULL);
		}
		// If number of components is greater than one, add SEC constraints
		if (ncomp >= 2) {
			for (int k = 1; k <= ncomp; k++) {

				int nnz = 0;
				double rhs = -1.0;
				
				int* index = (int *)malloc(inst->ncols * sizeof(int));
				double* value = (double *)malloc(inst->ncols * sizeof(double));

				char** cname = (char **)calloc(1, sizeof(char *));
				cname[0] = (char *)calloc(100, sizeof(char));

				if (index == NULL || value == NULL || xstar == NULL || cname == NULL || cname[0] == NULL) {
					print_error("Memory allocation error");
				}

				add_sec(&nnz, &rhs, k, index, value, cname, comp, inst);
				if (nnz > 0) {
					if (CPXaddrows(env, lp, 0, 1, nnz, &rhs, &sense, &izero, index, value, NULL, cname)) {
						print_error("Wrong CPXaddrows [degree]");
					}
				}

				free(cname[0]);
				free(cname);
				free(value);
				free(index);
			}
		}
		// Use patching heuristic to have a feasible solution
		if (ncomp >= 2) {
			patching_heuristic(succ, ncomp, comp, inst);
			reconstruct_sol(solution, succ, inst);
			plot_solution(inst, solution->path);
			if (VERBOSE > 1000) { printf("Solution cost before 2-Opt: %lf\n", solution->cost); }
			two_opt(solution, inst);
			check_sol(solution->path, solution->cost, inst);
			if (VERBOSE > 1000) { printf("Solution cost after 2-Opt: %lf\n", solution->cost); }
		}

        save_history_benders(objval, second()-inst->tstart, "../data/history_benders.txt");
        fflush(NULL);
	}

	if (iter <= 1) {
		print_error("Increase the time limit");
	}

	plot_cost_benders("../data/history_benders.txt", "../data/history_benders.png");

	free(comp);
	free(xstar);

	return 0;

}

/**
 * Post a heuristic solution to CPLEX
 * @param context CPLEX callback context pointer
 * @param inst instance containing the problem data
 * @param succ array containing the successors of each node in the solution
 * @param ncomp number of components in the solution
 * @param comp component array indicating the component each node belongs to
 */
void post_heuristic_solution(CPXCALLBACKCONTEXTptr context, instance *inst, int *succ, int ncomp, int *comp, double incumbent) {
    if (VERBOSE > 10000) {
        printf("Posting a heuristic solution\n");
    }

    patching_heuristic(succ, ncomp, comp, inst);

	tour* heuristic_solution = malloc(sizeof(tour));
    reconstruct_sol(heuristic_solution, succ, inst);
    two_opt(heuristic_solution, inst);
	// Check if the heuristic solution is better than the incumbent, if so, post it
	if (heuristic_solution->cost < incumbent) {

		if (VERBOSE > 2000) printf("Updating the incumbent from %lf to %lf\n", incumbent, heuristic_solution->cost);
		check_sol(heuristic_solution->path, heuristic_solution->cost, inst);

		int *succ_heuristics = (int *)malloc(inst->nnodes * sizeof(int));
		from_solution_to_succ(succ_heuristics, heuristic_solution, inst);

		double *xheu = (double *)calloc(inst->ncols, sizeof(double));
		for (int i = 0; i < inst->nnodes; i++) {
			xheu[xpos(i, succ_heuristics[i], inst)] = 1.0;
		}

		int *ind = malloc(inst->ncols * sizeof(int));
		for (int j = 0; j < inst->ncols; j++) {
			ind[j] = j;
		}

		if (CPXcallbackpostheursoln(context, inst->ncols, ind, xheu, heuristic_solution->cost, CPXCALLBACKSOLUTION_NOCHECK)) {
			print_error("CPXcallbackpostheursoln() error");
		}

		if (VERBOSE > 10000) {
			printf("Posted heuristic solution cost: %lf\n", heuristic_solution->cost);
			fflush(NULL);
		}

		free(ind);
		free(succ_heuristics);
		free(xheu);
	}
    free(heuristic_solution->path);
    free(heuristic_solution);
}


/**
 * Callback function for CPLEX to add cuts
 * @param context CPLEX callback context pointer
 * @param contextid CPLEX callback context ID
 * @param userhandle user-defined handle to pass user data
 * @return 0 on success, non-zero on failure
 */
int CPXPUBLIC candidate_callback(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle ) {
	instance* inst = (instance*) userhandle;

	char sense = 'L';
	int izero = 0;

	double* xstar = (double*) malloc(inst->ncols * sizeof(double));
	int *succ = (int*)malloc(inst->nnodes * sizeof(int));
	int *comp = (int *)malloc(inst->nnodes * sizeof(int));
	if (xstar == NULL || comp == NULL) {
		print_error("Memory allocation error for xstar");
	}

	double objval = CPX_INFBOUND;
	int ncomp = 0;
	if ( CPXcallbackgetcandidatepoint(context, xstar, 0, inst->ncols-1, &objval) ){
		print_error("CPXcallbackgetcandidatepoint error");
	}
	build_sol(xstar, inst, succ, comp, &ncomp);

	// Retrieve incumbent
	double incumbent;
	double x_dummy;
	CPXcallbackgetincumbent(context, &x_dummy, 0, 0, &incumbent);

	if (ncomp >= 2) {
		// Add SEC constraints for each component
		int* index = (int *)malloc(inst->ncols * sizeof(int));
		double* value = (double *)malloc(inst->ncols * sizeof(double));
		if (index == NULL || value == NULL) {
			print_error("Memory allocation error");
		}

		for (int k = 1; k <= ncomp; k++) {
			int nnz = 0;
			double rhs = -1.0;

			char** cname = (char **)calloc(1, sizeof(char *));
			cname[0] = (char *)calloc(100, sizeof(char));
			if (cname == NULL || cname[0] == NULL) {
				print_error("Memory allocation error for cname");
			}

			add_sec(&nnz, &rhs, k, index, value, cname, comp, inst);
			if (nnz > 0) {
				if ( CPXcallbackrejectcandidate(context, 1, nnz, &rhs, &sense, &izero, index, value) ) {
					print_error("CPXcallbackrejectcandidate() error"); // reject the solution and adds one cut
				}
			}
			free(cname[0]);
			free(cname);
		}

		free(value);
		free(index);

		// post_heuristic solution
		CPXLONG nodedepth;
		int status;

		int error = CPXcallbackgetinfoint(context, CPXCALLBACKINFO_CANDIDATE_SOURCE, &status);

		if(error){
			print_error("CPXgetcallbackinfo() error getting CANDIDATE_SOURCE");
		}

		if (inst->mode == 2 || inst->mode == 4 || inst->mode == 6) {
			post_heuristic_solution(context, inst, succ, ncomp, comp, incumbent);
		}
	}

	free(comp);
	free(xstar);

	return 0;
}

/**
 * Callback function for CPLEX to add usercuts
 * @param cut_val value of the cut
 * @param cut_nodes number of nodes in the cut
 * @param cut array containing the nodes in the cut
 * @param params user-defined parameters passed to the callback
 * @return
 */
int violated_cut_callback(double cut_val, int cut_nodes, int* cut, void* params) {

	pass_params* parameters = (pass_params*) params;
	char sense = 'L';
	double rhs = (double)cut_nodes - 1.0;
	int purgeable = CPX_USECUT_FILTER;
	int rmatbeg = 0;
	int local = 0;

	int *index = (int *)malloc((cut_nodes * cut_nodes) * sizeof(int));
	double *value = (double *)malloc((cut_nodes * cut_nodes) * sizeof(double));
	int nnz = 0;
	for (int i = 0; i < cut_nodes; i++) {
		for (int j = i+1; j < cut_nodes; j++) {
			index[nnz] = xpos(cut[i], cut[j], parameters->inst);
			value[nnz] = 1.0;
			nnz++;
		}
	}

	if(CPXcallbackaddusercuts(parameters->context, 1, nnz, &rhs, &sense, &rmatbeg, index, value, &purgeable, &local)) {print_error("Error on CPXcallbackaddusercuts()");}

	free(value);
	free(index);
	return 0;

}

/**
 * Calculate the violation of a cut
 * @param nnz number of non-zero entries in the cut
 * @param rhs right-hand side value of the cut
 * @param sense 'sense' of the cut
 * @param index array containing the indices of the variables in the cut
 * @param value array containing the coefficients of the variables in the cut
 * @param xstar solution vector from CPLEX
 * @return the violation of the cut
 */
double cut_violation(int nnz, double rhs, char sense, int* index, double* value, double* xstar) {
	double cut_val = 0.0;
	for (int i = 0; i < nnz; i++) {
		cut_val += value[i] * xstar[index[i]];
	}

	if (sense == 'L') {
		return fmax(0.0, cut_val - rhs);
	} else if (sense == 'G') {
		return fmax(0.0, rhs - cut_val);
	} else {
		return fabs(rhs - cut_val);
	}
	return 0;
}

/**
 * Callback function for CPLEX to add cuts
 * @param context CPLEX callback context pointer
 * @param contextid CPLEX callback context ID
 * @param userhandle user-defined handle to pass user data
 * @return
 */
int CPXPUBLIC relaxation_callback(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle ) {
	instance* inst = (instance*) userhandle;

	int ecount = 0;
	int* elist = (int*)malloc((inst->ncols * 2) * sizeof(int));
	if (elist == NULL) {
		print_error("Memory allocation error for elist");
	}
	// information of the node
	int current_thread = -1; 
	int current_node = -1;
	if (CPXcallbackgetinfoint(context, CPXCALLBACKINFO_THREADID, &current_thread))
		print_error("CPXcallbackgetinfoint error");
	if (CPXcallbackgetinfoint(context, CPXCALLBACKINFO_NODECOUNT, &current_node))
		print_error("CPXcallbackgetinfoint error");

	// Manage the number of times relaxation callback is applied
	if(current_node % inst->nnodes != 0)
		return 0;
	// Build the edge list for the complete graph, make it suitable for Concorde's functions
	int index = 0;
	for (int i = 0; i < inst->nnodes; i++) {
		for (int j = i+1; j < inst->nnodes; j++) {
			elist[index++] = i;
			elist[index++] = j;
			ecount++;
		}
	}

	double* xstar = (double*) malloc(inst->ncols * sizeof(double));
	if (xstar == NULL) {
		print_error("Memory allocation error for xstar");
	}
	double objval = CPX_INFBOUND;
	if ( CPXcallbackgetrelaxationpoint(context, xstar, 0, inst->ncols-1, &objval) ){
		print_error("CPXcallbackgetcandidatepoint error");
	}

	int ncomp;
	int *compscount = (int *)malloc(inst->nnodes * sizeof(int));
	int* comps = (int *)malloc(inst->nnodes * sizeof(int));
	if (compscount == NULL || comps == NULL) {
		print_error("Memory allocation error for compscount or comps");
	}

	// RETURNS the connected components of the graph given by the edgeset
	if (CCcut_connect_components(inst->nnodes, ecount, elist, xstar, &ncomp, &compscount, &comps)) {
		print_error("CCcut_connect_components error");
	}

	double cutoff = 1.9; // require a violation of at least 0.1 in the sec in the cut form
	pass_params params = {
		.context = context,
		.ecount = ecount,
		.elist = elist,
		.comp = comps,
		.ncomp = ncomp,
		.inst = inst,
		.xstar = xstar
	};

	if (ncomp == 1) {
		if (CCcut_violated_cuts(inst->nnodes, ecount, elist, xstar, cutoff, violated_cut_callback, &params)) {
			print_error("CCcut_violated_cuts error");
		}
	} else {
		int start = 0;

		// add sec for each components
		for(int c=0; c<ncomp; ++c) {
			// number of nodes in the current component
			int compsize = compscount[c];

			// current subtour in the graph
			int* subtour = (int*) calloc(compsize, sizeof(int));

			for(int i=0; i<compsize; ++i) {
				subtour[i] = comps[i+start];
			}

			double cutval = 0.0; // default value to pass checks
			violated_cut_callback(cutval, compsize, subtour, &params);

			start += compsize;

			free(subtour);
		}
	}

	free(comps);
	free(compscount);
	free(xstar);
	free(elist);

	return 0;
}

/**
 * Callback function for CPLEX to add cuts
 * @param context CPLEX callback context pointer
 * @param contextid CPLEX callback context ID
 * @param userhandle user-defined handle to pass user data
 * @return 0 on success, non-zero on failure
 */
int CPXPUBLIC sec_callback(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void* userhandle) {
	instance* inst = (instance*) userhandle;

	if(contextid == CPX_CALLBACKCONTEXT_CANDIDATE) {
		return candidate_callback(context, contextid, inst);
	}
	if(contextid == CPX_CALLBACKCONTEXT_RELAXATION) {
		return relaxation_callback(context, contextid, inst);
	}

	print_error("contextid unknown in callback");
	return 1;
}

/**
 * Branch and cut function
 * @param env CPLEX environment pointer
 * @param lp CPLEX LP pointer
 * @param contextid CPLEX callback context ID
 * @param inst instance containing the problem data
 * @param succ successor array to be filled
 * @param ncomp number of components found in the solution
 * @param solution tour structure to store the solution
 * @return 0 on success, non-zero on failure
 */
int branch_and_cut(CPXENVptr env, CPXLPptr lp, CPXLONG contextid, instance *inst, int *succ, int ncomp, tour* solution) {
	double objval = INF_COST;
	int *comp = (int *)malloc(inst->nnodes * sizeof(int));
	double *xstar = (double*)malloc(inst->ncols * sizeof(double));

	if(CPXcallbacksetfunc(env, lp, contextid, sec_callback, inst))
		print_error("CPXcallbacksetfunc() error");

	solver(env, lp, inst, xstar, succ, comp, &ncomp, &objval);
	if (VERBOSE > 1000) {
		printf("Lower Bound: %10.2lf, Number of components: %4d, Time: %5.2lfs\n", objval, ncomp, second()-inst->tstart);
		fflush(NULL);
	}

	if (ncomp >= 2) {
		patching_heuristic(succ, ncomp, comp, inst);
		reconstruct_sol(solution, succ, inst);
		two_opt(solution, inst);
		check_sol(solution->path, solution->cost, inst);
		printf("Solution cost %lf\n", solution->cost);
	}

	free(comp);
	free(xstar);
	return 0;

}

/**
 * Function to implement the patching heuristic to a solution
 * @param succ successor array containing the next node in the tour for each node
 * @param ncomp number of components in the solution
 * @param comp component array indicating the component each node belongs to
 * @param inst instance containing the problem data
 */
void patching_heuristic(int* succ, int ncomp, int* comp, instance* inst) {
    if (succ == NULL) { print_error("Error occurred while allocating memory for succ\n"); }
    if (comp == NULL) { print_error("Error occurred while allocating memory for comp\n"); }
    if (inst == NULL) { print_error("Error occurred while allocating memory for inst\n"); }

    while (ncomp > 1) {
        int best_i = -1;
        int best_j = -1;
        int best_cross = 0;
        double best_delta = INF_COST;

        for (int i = 0; i < inst->nnodes; i++) {
            for (int j = i + 1; j < inst->nnodes; j++) {
                if (comp[i] != comp[j]) {
                    double delta = compute_delta_straight(i, j, succ, inst);
                    if (delta + EPS_COST < best_delta) {
                        best_delta = delta;
                        best_i = i;
                        best_j = j;
                        best_cross = 0;
                    }
                    double delta_cross = compute_delta_cross(i, j, succ, inst);
                    if (delta_cross < best_delta + EPS_COST) {
                        best_delta = delta_cross;
                        best_i = i;
                        best_j = j;
                        best_cross = 1;
                    }
                }
            }
        }

        if (best_i < 0 || best_j < 0) {
            print_error("No valid patch found\n");
        }

        if (best_cross) {
			int temp = succ[best_i];
			succ[best_i] = succ[best_j];
			succ[best_j] = temp;

			for (int k = 0; k < inst->nnodes; k++) {
				if (comp[k] == comp[best_j] && k != best_j) {
					comp[k] = comp[best_i];
				}
			}
			comp[best_j] = comp[best_i];
			ncomp--;
        } else {
			int temp = succ[best_i];
			succ[best_i] = best_j;
			int past_node = succ[best_j];
			int current_node = succ[past_node];
			succ[past_node] = temp;

			// Reverse the component of best_j and succ[best_j]
			reverse_component(succ, best_j, current_node, past_node);

			for (int k = 0; k < inst->nnodes; k++) {
				if (comp[k] == comp[best_j] && k != best_j) {
					comp[k] = comp[best_i];
				}
			}
			comp[best_j] = comp[best_i];
			ncomp--;
        }
    }

    if (VERBOSE > 2000) {
        printf("Finished patching heuristic\n");
    }
}

/**
 * Solve the TSP problem using CPLEX
 * @param inst instance containing the problem data
 * @param alg algorithm to be used for solving the TSP problem
 * @return 0 on success, non-zero on failure
 */
int TSPopt(instance *inst, int alg) {
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

	inst->ncols = CPXgetnumcols(env, lp);
	int ncomp = 9999;
	int *succ = (int *)malloc(inst->nnodes * sizeof(int));
	// Flag to indicate if 2-opt is applied. Will be set to 1 if
	// CPLEX is not able to solve the problem to the optimality
	int two_opt_flag = 0;
	tour* solution = (tour *)malloc(sizeof(tour));
	CPXLONG contextid = -1000;
	if (inst->mode == 3 || inst->mode == 5 || inst->mode == 6) {
		contextid = CPX_CALLBACKCONTEXT_CANDIDATE | CPX_CALLBACKCONTEXT_RELAXATION;
	} else{
		contextid = CPX_CALLBACKCONTEXT_CANDIDATE;
	}

	if (alg == 6){
		branch_and_cut(env, lp, contextid, inst, succ, ncomp, solution);
	} else {
		benders(env, lp, inst, succ, ncomp, solution);
	}
	
	#ifdef DEBUG
		check_degrees(inst, xstar);	
	#endif

	reconstruct_sol(solution, succ, inst);
	update_best_sol(inst, solution);
	plot_solution(inst, solution->path);

	// Plot the solution path of both CPLEX
    plot_succ_path(succ, inst, "../data/solution_cplex.png");


	// Free allocated memory
	free(solution->path);
	free(solution);
    free(succ);

    // Free and close CPLEX model
    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);

    return 0;
}