#include <matheuristics.h>

double min(double a, double b) {
    return (a < b) ? a : b;
}

int hard_fixing(instance* inst) {

    tour* solution = malloc(sizeof(tour));
    solution->path = malloc((inst->nnodes + 1) * sizeof(int));
    solution->cost = 0.0;

    if (greedy(0, solution, 1, inst)) { print_error("Error during heuristic\n");}
    vns(inst, solution, inst->timelimit * 0.1, 3);
    if (VERBOSE > 1000) {
        printf("Initial solution cost: %lf, found after %5.2lf second\n", solution->cost, second() - inst->tstart);
        fflush(NULL);
    }

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
    inst->ncols = CPXgetnumcols(env, lp);


    if (CPXsetintparam(env, CPX_PARAM_SCRIND, 1)) // Enable screen output
        print_error("CPXsetintparam() error");
    if (CPXsetintparam(env, CPX_PARAM_MIPDISPLAY, 2)) // Verbosity level
        print_error("CPXsetintparam() error");
    if (CPXsetdblparam(env, CPX_PARAM_EPGAP, 1e-7)) // Set the optimality gap
        print_error("CPXsetdblparam() error");
    if (CPXsetintparam(env, CPXPARAM_RandomSeed, inst->seed))
        print_error("CPXsetintparam() error");

    double *xstar = calloc(inst->ncols, sizeof(double));
    int* succ = malloc(sizeof(int) * inst->nnodes);
    from_solution_to_succ(succ, solution, inst);
    for (int i = 0; i < inst->nnodes; i++) { xstar[xpos(i, succ[i], inst)] = 1.0; }
    double p = 0.5;
    double residual_time = inst->timelimit - (inst->timelimit * 0.1);

    while (true) {
        if (second() - inst->tstart > residual_time) {
            printf("Time limit reached\n");
            break;
        }
        int* fixed_edges = calloc(inst->nnodes,sizeof(int));

        int cnt = 0;
        for (int i = 0; i < inst->nnodes; i++) {
            if (random01() > p) {
                fixed_edges[cnt] = xpos(i, succ[i], inst);
            }
        }

        double current_residual_time = inst->timelimit - (second() - inst->tstart);
        char* lu = calloc(cnt,inst->nnodes);
        double* bd = calloc(cnt, sizeof(double));
        memset(lu, 'L', cnt);
        memset(bd, 1.0, cnt);

        CPXchgbds(env, lp, cnt, fixed_edges, lu, bd);
        if (CPXsetdblparam(env, CPX_PARAM_TILIM, min(current_residual_time, inst->timelimit / 20)))
            print_error("CPXsetdblparam() error");
        //64961.000000
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
            check_sol(solution->path, solution->cost, inst);
            solution->cost = objval;
            for (int i = 0; i < inst->nnodes; i++) {
                if (xstar[xpos(solution->path[i], solution->path[i + 1], inst)] > 0.5) {
                    succ[solution->path[i]] = solution->path[i + 1];
            }
            }

            // Reconstruct the solution path from the successors array and check its feasibility
            reconstruct_sol(solution, succ, inst);

            if (VERBOSE > 1000) {
                printf("New solution cost: %lf\n found after %5.2lf second\n", solution->cost, second() - inst->tstart);
                fflush(NULL);
            }
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