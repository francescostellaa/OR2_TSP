#include <matheuristics.h>

// Function to return the maximum of two doubles
double min(double a, double b) {
    return (a < b) ? a : b;
}

#include <cplex.h>

/**
 * Initialize the CPLEX environment and build the model for the TSP instance
 * @param inst instance containing the problem data
 * @param env pointer to the CPLEX environment
 * @param lp pointer to the CPLEX problem
 */
void initialize_cplex(instance* inst, CPXENVptr* env, CPXLPptr* lp) {
    int error = 0;

    *env = CPXopenCPLEX(&error);
    if (*env == NULL) {
        char errormsg[CPXMESSAGEBUFSIZE];
        CPXgeterrorstring(*env, error, errormsg);
        print_error(errormsg);
    }

    *lp = CPXcreateprob(*env, &error, "TSP");
    if (*lp == NULL) {
        char errormsg[CPXMESSAGEBUFSIZE];
        CPXgeterrorstring(*env, error, errormsg);
        print_error(errormsg);
    }

    build_model(inst, *env, *lp);
    inst->ncols = CPXgetnumcols(*env, *lp);

    if (CPXsetintparam(*env, CPX_PARAM_SCRIND, 1)) // Enable screen output
        print_error("CPXsetintparam() error");
    if (CPXsetintparam(*env, CPX_PARAM_MIPDISPLAY, 2)) // Verbosity level
        print_error("CPXsetintparam() error");
    if (CPXsetdblparam(*env, CPX_PARAM_EPGAP, 1e-7)) // Set the optimality gap
        print_error("CPXsetdblparam() error");
    if (CPXsetintparam(*env, CPXPARAM_RandomSeed, inst->seed))
        print_error("CPXsetintparam() error");
}

/**
 * This function implements the hard-fixing matheuristic for solving the TSP problem.
 * @param inst instance containing the problem data
 * @param params parameters for the algorithm
 * @return zero on success, non-zero on failures
 */
int hard_fixing(instance* inst, parameters* params) {

    // Check if probability of hard fixing is within the valid range
    if (params->prob_hard_fixing < 0.2 || params->prob_hard_fixing > 0.8) { print_error("Fixing probability cannot be greater than 0.8 or less than 0.2!\n"); }

    // Initialize temporary solution
    tour* solution = malloc(sizeof(tour));
    solution->path = malloc((inst->nnodes + 1) * sizeof(int));
    solution->cost = 0.0;

    // Compute the initial heuristic solution using VNS algorithm
    if (greedy(0, solution, 1, inst)) { print_error("Error during heuristic\n");}
    vns(inst, solution, inst->timelimit * 0.1, 3);
    if (VERBOSE > 1000) {
        printf("Initial solution cost: %lf, found after %5.2lf second\n", solution->cost, second() - inst->tstart);
        fflush(NULL);
    }

    // Initialize CPLEX environment and the problem
    CPXENVptr env;
    CPXLPptr lp;
    initialize_cplex(inst, &env, &lp);
    inst->ncols = CPXgetnumcols(env, lp);
    // Set context id to install both candidate and relaxation callbacks
    CPXLONG contextid = CPX_CALLBACKCONTEXT_CANDIDATE | CPX_CALLBACKCONTEXT_RELAXATION;

    // Build succ vector from VNS solution
    double *xstar = calloc(inst->ncols, sizeof(double));
    int* succ = malloc(sizeof(int) * inst->nnodes);
    from_solution_to_succ(succ, solution, inst);
    for (int i = 0; i < inst->nnodes; i++) { xstar[xpos(i, succ[i], inst)] = 1.0; }
    if (VERBOSE > 2000) {printf("Prob hard fixing: %lf\n", params->prob_hard_fixing);}
    double p = params->prob_hard_fixing;
    double residual_time = inst->timelimit - (inst->timelimit * 0.1);
    int error = 0;

    // Set VNS solution as mipstart solution for CPLEX
    double *xheu = calloc(inst->ncols, sizeof(double));  // all zeros, initially
    for ( int i = 0; i < inst->nnodes; i++ ) xheu[xpos(i,succ[i],inst)] = 1.0;
    int *ind = malloc(inst->ncols * sizeof(int));
    for ( int j = 0; j < inst->ncols; j++ ) ind[j] = j;

    // Use CPXaddmipstarts to add the initial solution from VNS as a MIP start
    int effortlevel = CPX_MIPSTART_NOCHECK;
    int beg = 0;
    if ( CPXaddmipstarts(env, lp, 1, inst->ncols, &beg, ind, xheu, &effortlevel, NULL)) {
        print_error("CPXaddmipstarts() error");
    }
    free(ind);
    free(xheu);
    // Install callbacks
    if(CPXcallbacksetfunc(env, lp, contextid, sec_callback, inst))
        print_error("CPXcallbacksetfunc() error");
    
    // Set the node limit to 1000 nodes
    if (CPXsetlongparam(env, CPX_PARAM_NODELIM, 1000)) { print_error("CPXsetlongparam() NODELIM error"); }

    save_history_cost(solution->cost, "../data/cost_hard_fixing.txt");
    int iteration = 0;
    while (true) {
        iteration++;
        if (second() - inst->tstart > residual_time) {
            printf("Time limit reached\n");
            break;
        }
        
        int* fixed_edges = calloc(inst->nnodes,sizeof(int));
        int ncomp = 9999;
        int cnt = 0;
        for (int i = 0; i < inst->nnodes; i++) {
            if (random01() > p) {
                fixed_edges[cnt++] = xpos(i, succ[i], inst);
            }
        }

        // Set lower bounds of selected edges to 1.0
        if (cnt == 0) {
            if (VERBOSE > 100) {printf("No edges to fix\n");}
            free(fixed_edges);
            continue;
        }
        char* lu = malloc(cnt * sizeof(char));
        double* bd = malloc(cnt * sizeof(double));
        for (int i = 0; i < cnt; i++) {
            lu[i] = 'L';
            bd[i] = 1.0;
        }
        CPXchgbds(env, lp, cnt, fixed_edges, lu, bd);

        double current_residual_time = inst->timelimit - (second() - inst->tstart);
        // Set the time limit to the resiual time
        if (CPXsetdblparam(env, CPX_PARAM_TILIM, current_residual_time))
            print_error("CPXsetdblparam() error");

        // Solve the model with the fixed edges
        if (CPXmipopt(env, lp)) {
            char errormsg[CPXMESSAGEBUFSIZE];
            CPXgeterrorstring(env, error, errormsg);
            print_error(errormsg);
        }
        double objval = INF_COST;
        if (CPXsolution(env, lp, &error, &objval, xstar, NULL, NULL, NULL)) {
            char errormsg[CPXMESSAGEBUFSIZE];
            CPXgeterrorstring(env, error, errormsg);
            print_error(errormsg);
        }

        // Update the solution if the new objective value is better
        if (objval + EPS_COST < solution->cost) {
            solution->cost = objval;
            int *comp = (int *)malloc(inst->nnodes * sizeof(int));
            build_sol(xstar, inst, succ, comp, &ncomp);

            // Reconstruct the solution path from the successors array and check its feasibility
            reconstruct_sol(solution, succ, inst);

            if (VERBOSE > 1000) {
                printf("New solution cost: %lf found after %5.2lf seconds\n", solution->cost, second() - inst->tstart);
                fflush(NULL);
            }
            free(comp);
        }

        save_history_cost(solution->cost, "../data/cost_hard_fixing.txt");

        // Reset the lower bounds
        for (int i = 0; i < cnt; i++) {
            bd[i] = 0.0;
        }
        if (CPXchgbds(env, lp, cnt, fixed_edges, lu, bd)) {
            print_error("CPXchgbds() error resetting lower bounds");
        }
        free(bd);
        free(lu);
        free(fixed_edges);
        if (VERBOSE > 500) {printf("Iteration %i, cost: %lf\n", iteration, solution->cost);}
    }

    // Update the best solution
    update_best_sol(inst, solution);
    if (VERBOSE > 1) { printf("Best cost: %lf\n", inst->best_sol->cost); }
    // Free memory
    free(succ);
    free(xstar);
    free(solution->path);
    free(solution);
    // Free and close CPLEX model
    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);

    plot_history_cost("../data/cost_hard_fixing.txt", "../data/hard_fixing.png");

    return 0;
}

/**
 * This function implements the local branching matheuristic for solving the TSP problem.
 * @param inst instance containing the problem data
 * @param parameters parameters for the local branching algorithm
 * @return zero on success, non-zero on failures
 */
int local_branching(instance* inst, parameters* parameters) {
    // Check if k_neighborhood is within the valid range
    if (parameters->k_neighborhood > inst->nnodes) {
        print_error("Error: k must be smaller than the number of nodes of the graph\n");
    }
    // Initialize temporary solution
    tour* solution = malloc(sizeof(tour));
    solution->path = malloc((inst->nnodes + 1) * sizeof(int));
    solution->cost = 0.0;

    // Compute the initial heuristic solution using VNS algorithm
    if (greedy(0, solution, 1, inst)) { print_error("Error during heuristic\n");}
    vns(inst, solution, inst->timelimit * 0.1, 3);
    if (VERBOSE > 1000) {
        printf("Initial solution cost: %lf, found after %5.2lf second\n", solution->cost, second() - inst->tstart);
        fflush(NULL);
    }

    // Initialize CPLEX environment and the problem
    CPXENVptr env;
    CPXLPptr lp;
    initialize_cplex(inst, &env, &lp);

    inst->ncols = CPXgetnumcols(env, lp);
    // Set context id to install both candidate and relaxation callbacks
    CPXLONG contextid = CPX_CALLBACKCONTEXT_CANDIDATE | CPX_CALLBACKCONTEXT_RELAXATION;

    // Build succ vector from VNS solution
    double *xstar = calloc(inst->ncols, sizeof(double));
    int* succ = malloc(sizeof(int) * inst->nnodes);
    from_solution_to_succ(succ, solution, inst);
    for (int i = 0; i < inst->nnodes; i++) { xstar[xpos(i, succ[i], inst)] = 1.0; }
    int k = parameters->k_neighborhood;
    double residual_time = inst->timelimit - (inst->timelimit * 0.1);
    int error = 0;

    // Set VNS solution as mipstart solution for CPLEX
    double *xheu = calloc(inst->ncols, sizeof(double));  // all zeros, initially
    for ( int i = 0; i < inst->nnodes; i++ ) xheu[xpos(i,succ[i],inst)] = 1.0;
    int *ind = malloc(inst->ncols * sizeof(int));
    for ( int j = 0; j < inst->ncols; j++ ) ind[j] = j;

    int effortlevel = CPX_MIPSTART_NOCHECK;
    int beg = 0;
    if ( CPXaddmipstarts(env, lp, 1, inst->ncols, &beg, ind, xheu, &effortlevel, NULL)) {
        print_error("CPXaddmipstarts() error");
    }
    free(ind);
    free(xheu);

    if(CPXcallbacksetfunc(env, lp, contextid, sec_callback, inst))
        print_error("CPXcallbacksetfunc() error");
    
    // Set the node limit to 1000 nodes
    if (CPXsetlongparam(env, CPX_PARAM_NODELIM, 1000))
        print_error("CPXsetlongparam() NODELIM error");

    save_history_cost(solution->cost, "../data/cost_local_branching.txt");
    int iteration = 0;

    while(true) {
        iteration++;
        if (second() - inst->tstart > residual_time) {
            printf("Time limit reached\n");
            break;
        }

        // Create a new row for the local branching constraint
        int izero = 0;
        char sense = 'G';
        double rhs = (double) (inst->nnodes - k);   
        int nnz = 0;
        int max_edges = (inst->nnodes * (inst->nnodes - 1)) / 2;
        int* index = malloc(max_edges * sizeof(int));
        double* value = malloc(max_edges * sizeof(double));

		if (index == NULL || value == NULL) {
			print_error("Memory allocation error");
		}

        for (int i = 0; i < inst->nnodes; i++) {
            for (int j = i + 1; j < inst->nnodes; j++) {
                if (i == j) { continue; }
                if (xstar[xpos(i, j, inst)] > 0.5) {
                    index[nnz] = xpos(i, j, inst);
                    value[nnz] = 1.0;
                    nnz++;
                }
            }
        }
        if ( CPXaddrows(env, lp, 0, 1, nnz, &rhs, &sense, &izero, index, value, NULL, NULL) ) {
            print_error("CPXaddrows() error");
        }
        // Set the time limit to the resiual time
        double current_residual_time = inst->timelimit - (second() - inst->tstart);
        if (CPXsetdblparam(env, CPX_PARAM_TILIM, min(current_residual_time, inst->timelimit * 0.1)))
            print_error("CPXsetdblparam() error");
        // Solve the model with the local branching constraint
        if (CPXmipopt(env, lp)) {
            char errormsg[CPXMESSAGEBUFSIZE];
            CPXgeterrorstring(env, error, errormsg);
            print_error(errormsg);
        }
        double objval = INF_COST;
        if (CPXsolution(env, lp, &error, &objval, xstar, NULL, NULL, NULL)) {
            char errormsg[CPXMESSAGEBUFSIZE];
            CPXgeterrorstring(env, error, errormsg);
            print_error(errormsg);
        }

        // Check if the solution is optimal or CPLEX reached the time limit to
        // increase / decrease the neighborhood size
        if (CPXgetstat(env, lp) == CPXMIP_OPTIMAL || CPXgetstat(env, lp) == CPXMIP_NODE_LIM_FEAS) {
            if (VERBOSE > 500) {printf("Increasing neighborhood size to %d\n", k + 1);}
            k++;
            fflush(NULL);
            k++;
        }
        else if (CPXgetstat(env, lp) == CPXMIP_TIME_LIM_FEAS) {
            if (k > 1) {
                if (VERBOSE > 500)  printf("Decreasing neighborhood size to %d\n", k - 1);
                fflush(NULL);
                k--;
            } else {
                if (VERBOSE > 500) printf("No solution found, stopping local branching\n");
                free(value);
                free(index);
                break;
            }
        } else {
            if (k > 1) {
                if (VERBOSE > 500)  printf("No solution found, decreasing neighborhood size to %d\n", k - 1);
                fflush(NULL);
                k--;
            } else {
                if (VERBOSE > 500) printf("No solution found, stopping local branching\n");
                free(value);
                free(index);
                break;
            }
        }

        // Update the solution if the new objective value is better
        int ncomp = 9999;
        if (objval + EPS_COST < solution->cost) {
            solution->cost = objval;
            int *comp = (int *)malloc(inst->nnodes * sizeof(int));
            build_sol(xstar, inst, succ, comp, &ncomp);

            // Reconstruct the solution path from the successors array and check its feasibility
            reconstruct_sol(solution, succ, inst);

            if (VERBOSE > 1000) {
                printf("New solution cost: %lf found after %5.2lf seconds\n", solution->cost, second() - inst->tstart);
                fflush(NULL);
            }
            free(comp);
        }

        int num_rows = CPXgetnumrows(env, lp);
        if ( CPXdelrows(env, lp, num_rows - 1, num_rows - 1) ) {
            print_error("CPXdelrows() error");
        }

        save_history_cost(solution->cost, "../data/cost_local_branching.txt");

        free(value);
        free(index);
        if (VERBOSE > 500) printf("Iteration %d completed, current best cost: %lf, time: %lf\n", iteration, solution->cost, second() - inst->tstart);
    }

    update_best_sol(inst, solution);
    if (VERBOSE > 1) { printf("Best cost: %lf\n", inst->best_sol->cost); }
    free(succ);
    free(xstar);
    free(solution->path);
    free(solution);
    // Free and close CPLEX model
    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);

    plot_history_cost("../data/cost_local_branching.txt", "../data/local_branching.png");
    
    return 0;

}