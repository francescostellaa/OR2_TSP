#include <greedy.h>


/**
 * Greedy algorithm to solve the TSP
 * @param initial_point initial point to start the greedy algorithm
 * @param inst instance with the nodes
 * @param run_2opt flag to run the 2-opt refinement
 */
int greedy(int initial_point, instance* inst, int run_2opt){
    int n = inst->nnodes;
    //int* solution = (int*)malloc((n+1) * sizeof(int));
    tour* solution = (tour*)malloc(sizeof(tour));
    solution->path = (int*)malloc((n+1) * sizeof(int));
    solution->cost = 0.0;
    for (int i = 0; i < n; i++) {
        solution->path[i] = i;
    }
    swap(solution->path, 0, initial_point);

    for (int i = 0; i < n - 1; i++) {
        double min_cost = INF_COST;
        int min_index = -1;
        for (int j = i + 1; j < n; j++) {
            double current_cost = inst->cost_matrix[solution->path[i] * n + solution->path[j]];
            if (current_cost < min_cost) {
                min_cost = current_cost;
                min_index = j;
            }
        }
        solution->cost += min_cost;
        swap(solution->path, i + 1, min_index);
    }
    solution->path[n] = solution->path[0];
    solution->cost += inst->cost_matrix[solution->path[n-1] * n + solution->path[n]];
    save_history_cost(solution->cost);
    update_best_sol(inst, solution);
    
    if (run_2opt){
        two_opt(solution, inst);
    }
    
    free(solution);

    return 0;
}

/**
 * Multi-start greedy algorithm to solve the TSP
 * repeated greedy algorithm with different initial points
 * @param inst instance with the nodes
 */
int greedy_multi_start(instance* inst, int run_2opt) {

    int n = inst->nnodes;

    for (int i = 1; i < n; i++) {
        if(second() - inst->tstart > inst->timelimit) {
            if ( VERBOSE >= 100 ) { printf("Time limit reached\n"); }
            break;
        }
        greedy(i, inst, run_2opt);
        save_history_incumbent(inst->best_sol->cost);
    }

    plot_solution(inst, inst->best_sol->path);
    plot_incumbent();
    plot_history_cost();
    if(VERBOSE >= 1) { printf("Best cost: %lf\n", inst->best_sol->cost); }

    return 0;
}