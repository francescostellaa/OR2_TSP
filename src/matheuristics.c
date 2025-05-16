#include <matheuristics.h>

double min(double a, double b) {
    return (a < b) ? a : b;
}

#include <cplex.h>

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

    if (CPXsetintparam(*env, CPX_PARAM_SCRIND, 0)) // Enable screen output
        print_error("CPXsetintparam() error");
    if (CPXsetintparam(*env, CPX_PARAM_MIPDISPLAY, 2)) // Verbosity level
        print_error("CPXsetintparam() error");
    if (CPXsetdblparam(*env, CPX_PARAM_EPGAP, 1e-7)) // Set the optimality gap
        print_error("CPXsetdblparam() error");
    if (CPXsetintparam(*env, CPXPARAM_RandomSeed, inst->seed))
        print_error("CPXsetintparam() error");
}


int hard_fixing(instance* inst) {

    printf("PROB %lf", inst->prob_hard_fixing);
    if (inst->prob_hard_fixing < 0.2 || inst->prob_hard_fixing > 0.8) { print_error("Fixing probability cannot be greater than 0.8 or less than 0.2!\n"); }

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

    // Flag to indicate if 2-opt is applied. Will be set to 1 if
    // CPLEX is not able to solve the problem to the optimality

    CPXLONG contextid = -1000;
    contextid = CPX_CALLBACKCONTEXT_CANDIDATE | CPX_CALLBACKCONTEXT_RELAXATION;
    //contextid = CPX_CALLBACKCONTEXT_CANDIDATE;
    // Set maximum number of possible nodes in the braching tree
    // CPXsetlongparam(env, CPX_PARAM_NODELIM, 10);

    // Build succ vector from VNS solution
    double *xstar = calloc(inst->ncols, sizeof(double));
    int* succ = malloc(sizeof(int) * inst->nnodes);
    from_solution_to_succ(succ, solution, inst);
    for (int i = 0; i < inst->nnodes; i++) { xstar[xpos(i, succ[i], inst)] = 1.0; }
    double p = inst->prob_hard_fixing;
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

    while (true) {
        if (second() - inst->tstart > residual_time) {
            printf("Time limit reached\n");
            break;
        }
        int* fixed_edges = calloc(inst->nnodes,sizeof(int));
        int ncomp = 9999;
        int cnt = 0;
        for (int i = 0; i < inst->nnodes; i++) {
            if (random01() > p) {
                fixed_edges[cnt] = xpos(i, succ[i], inst);
            }
        }

        double current_residual_time = inst->timelimit - (second() - inst->tstart);

        // Set lower bounds of selected edges to 1.0
        char* lu = calloc(cnt,inst->nnodes);
        double* bd = calloc(cnt, sizeof(double));
        memset(lu, 'L', cnt);
        memset(bd, 1.0, cnt);
        CPXchgbds(env, lp, cnt, fixed_edges, lu, bd);

        // Set the time limit to the resiual time
        if (CPXsetdblparam(env, CPX_PARAM_TILIM, min(current_residual_time, inst->timelimit * 0.1)))
            print_error("CPXsetdblparam() error");

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
        //237134.893004
        // Update the solution if the new objective value is better
        if (objval + EPS_COST < solution->cost) {
            //check_sol(solution->path, solution->cost, inst);
            solution->cost = objval;
            int *comp = (int *)malloc(inst->nnodes * sizeof(int));
            build_sol(xstar, inst, succ, comp, &ncomp);
            /*for (int i = 0; i < inst->nnodes; i++) {
                if (xstar[xpos(solution->path[i], solution->path[i + 1], inst)] > 0.5) {
                    succ[solution->path[i]] = solution->path[i + 1];
                }
            }*/

            // Reconstruct the solution path from the successors array and check its feasibility
            reconstruct_sol(solution, succ, inst);

            if (VERBOSE > 1000) {
                printf("New solution cost: %lf\n found after %5.2lf second\n", solution->cost, second() - inst->tstart);
                fflush(NULL);
            }
            free(comp);
        }

        // Reset the lower bounds
        memset(bd, 0.0, cnt);
        CPXchgbds(env, lp, cnt, fixed_edges, lu, bd);
        free(bd);
        free(lu);
        free(fixed_edges);

    }

    // Update the best solution
    update_best_sol(inst, solution);
    if (VERBOSE > 1) { printf("Best cost: %lf\n", inst->best_sol->cost); }
    // Free memory
    free(succ);
    free(xstar);
    free(solution->path);
    free(solution);

    return 0;
}