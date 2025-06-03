#include <greedy.h>

/**
 * Multi-start greedy algorithm to solve the TSP
 * repeated greedy algorithm with different initial points
 * @param inst instance with the nodes
 * @param run_2opt flag to run the 2-opt refinement. If 1, the 2-opt refinement is applied
 * @param timelimit
 */
int greedy_multi_start(instance* inst, int run_2opt, double timelimit) {

    if (inst == NULL) {
        print_error("Error in initialization of the instance\n");
        return -1;
    }
    if (inst->best_sol == NULL) {
        printf("No solution exists.\n");
        return -1;
    }

    if (inst->best_sol->path == NULL) {
        print_error("Error occurred while allocating memory for the instance\n");
        return -1;
    }

    int n = inst->nnodes;

    for (int i = 1; i < n; i++) {
        if(second() - inst->tstart > timelimit) {
            if ( VERBOSE >= 100 ) { printf("Time limit reached\n"); }
            break;
        }
        tour* solution = malloc(sizeof(tour));
        if (solution == NULL) {
            print_error("Error: Memory allocation failed for solution\n");
            return -1;
        }

        solution->path = (int*)malloc((n + 1) * sizeof(int));
        solution->cost = 0.0;

        if (solution->path == NULL) {
            print_error("Error: Memory allocation failed for solution path\n");
            free(solution);
            return -1;
        }
        if (greedy(i, solution, run_2opt, inst) == 0) {
            update_best_sol(inst, solution);
        }
        save_history_cost(solution->cost, "../data/cost_greedy.txt");
        save_history_incumbent(inst->best_sol->cost, "../data/incumbent_greedy.txt");
        free(solution->path);
        free(solution);
    }

    plot_solution(inst, inst->best_sol->path);
    plot_incumbent("../data/incumbent_greedy.txt", "../data/incumbent_greedy.png");
    plot_history_cost("../data/cost_greedy.txt", "../data/cost_greedy.png");
    plot_incumbent_and_costs("../data/cost_greedy.txt", "../data/incumbent_greedy.txt", "../data/incumbent_and_costs_greedy.png");
    if(VERBOSE >= 1) { printf("Best cost: %lf\n", inst->best_sol->cost); }

    return 0;
}

/**
 * @brief
 * Greedy algorithm to solve the TSP
 * @param initial_point initial point to start the greedy algorithm
 * @param solution struct that con
 * @param run_2opt flag to run the 2-opt refinement
 * @param inst instance with the nodes
 */
int greedy(int initial_point, tour* solution, int run_2opt, const instance* inst){

    if (solution == NULL) {
        print_error("Error in greedy\n");
        return -1;
    }

    if (solution->path == NULL) {
        print_error("Error in greedy\n");
        return -1;
    }

    const int n = inst->nnodes;
    for (int i = 0; i < inst->nnodes; i++) {
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

    if (run_2opt){
        two_opt(solution, inst);
    }
    
    return 0;
}
