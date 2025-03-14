#include <greedy.h>


/**
 * Greedy algorithm to solve the TSP
 * @param initial_point initial point to start the greedy algorithm
 * @param inst instance with the nodes
 * @param run_2opt flag to run the 2-opt refinement
 */
int greedy(int initial_point, instance* inst, int run_2opt){
    int n = inst->nnodes;
    int* solution = (int*)malloc((n+1) * sizeof(int));
    
    double new_cost = 0;

    for (int i = 0; i < n; i++) {
        solution[i] = i;
    }
    swap(solution, 0, initial_point);

    for (int i = 0; i < n - 1; i++) {
        double min_cost = INF_COST;
        int min_index = -1;
        for (int j = i + 1; j < n; j++) {
            double current_cost = inst->cost[solution[i] * n + solution[j]];
            if (current_cost < min_cost) {
                min_cost = current_cost;
                min_index = j;
            }
        }
        new_cost += min_cost;
        swap(solution, i + 1, min_index);
    }
    solution[n] = solution[0];
    new_cost += inst->cost[solution[n-1] * n + solution[n]];

    update_best_sol(inst, solution, new_cost);
    
    if (run_2opt){
        two_opt(inst->best_sol, inst);
    }
    
    free(solution);

    return 0;

}


int greedy_multi_start(instance* inst) {

    inst->tstart = second();
    if ( VERBOSE >= 1000) { printf("Time start: %lf\n", inst->tstart); }

    int n = inst->nnodes;

    for (int i = 0; i < n; i++) {
        if(second() - inst->tstart > inst->timelimit) {
            if ( VERBOSE >= 100 ) { printf("Time limit reached\n"); }
            break;
        }
        if (greedy(i, inst, 1)){
            print_error("Error in greedy algorithm");
        }
        printf("Iteration: %d, Best Cost: %.6f\n", inst->num_iterations, inst->best_cost);
        save_history_incumbent(inst->num_iterations, inst->best_cost);
        inst->num_iterations++;
    }

    plot_solution(inst, inst->best_sol);
    plot_incumbent();
    if(VERBOSE >= 1) { printf("Best cost: %lf\n", inst->best_cost); }

    return 0;
}