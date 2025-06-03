#include <extra_mileage.h>

/**
 * This function implements the Extra Mileage algorithm for solving the TSP.
 * It runs the algorithm multiple times starting from different nodes.
 * @param inst instance with the nodes
 * @param run_2opt flag to run the 2-opt refinement. If 1, the 2-opt refinement is applied
 * @param timelimit time limit for the algorithm
 * @return zero on success, non-zero on failures
 */
int extra_mileage_multi_start(instance* inst, int run_2opt, double timelimit) {
    
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
        
        if (extra_mileage(i, solution, run_2opt, inst) == 0) {
            update_best_sol(inst, solution);
        }
        
        save_history_cost(solution->cost, "../data/NN/cost_extra_mileage.txt");
        save_history_incumbent(inst->best_sol->cost, "../data/NN/incumbent_extra_mileage.txt");
        
        free(solution->path);
        free(solution);
    }

    plot_solution(inst, inst->best_sol->path);
    plot_incumbent("../data/NN/incumbent_extra_mileage.txt", "../data/NN/incumbent_extra_mileage.png");
    plot_history_cost("../data/NN/cost_extra_mileage.txt", "../data/NN/cost_extra_mileage.png");
    plot_incumbent_and_costs("../data/NN/cost_extra_mileage.txt", "../data/NN/incumbent_extra_mileage.txt", "../data/NN/incumbent_and_costs_extra_mileage.png");

    if(VERBOSE >= 1) { printf("Best cost: %lf\n", inst->best_sol->cost); }

    return 0;
}

/**
 * This function implements the Extra Mileage algorithm for solving the TSP.
 * It starts from a given node and builds a tour by adding nodes that minimize the cost.
 * @param start_node The starting node for the tour
 * @param solution The tour to be constructed
 * @param run_2opt Whether to run 2-opt optimization on the solution
 * @param inst The instance containing the problem data
 * @return 0 on success, -1 on failure
 */
int extra_mileage(int start_node, tour* solution, int run_2opt, instance* inst) {

    if (solution == NULL || solution->path == NULL) {
        print_error("Error in extra mileage\n");
        return -1;
    }

    const int n = inst->nnodes;
    int* visited = (int*) calloc(n, sizeof(int));
    if (visited == NULL) {
        print_error("Memory allocation failed\n");
        return -1;
    }

    int min_idx = -1;
    double best_delta = INF_COST;

    for (int i = 0; i < n; i++) {
        if (i == start_node) continue;
        double cost = inst->cost_matrix[start_node * n + i];
        if (cost < best_delta) {
            best_delta = cost;
            min_idx = i;
        }
    }

    solution->path[0] = start_node;
    solution->path[1] = min_idx;
    solution->path[2] = start_node;
    solution->cost = best_delta + inst->cost_matrix[min_idx * n + start_node];

    visited[start_node] = 1;
    visited[min_idx] = 1;
    int tour_size = 2;

    while (tour_size < n) {
        int best_node = -1; 
        best_delta = INF_COST;
        min_idx = -1;

        for (int i = 0; i < n; i++) {
            
            if (visited[i]) continue;

            for (int j = 0; j < tour_size; j++) {

                double delta = inst->cost_matrix[solution->path[j] * n + i] 
                                + inst->cost_matrix[i * n + solution->path[j + 1]] 
                                - inst->cost_matrix[solution->path[j] * n + solution->path[j + 1]];
                
                if (delta < best_delta) {
                    best_delta = delta;
                    min_idx = j + 1;
                    best_node = i;   
                }

            }
            
        }

        for (int i = tour_size + 1; i > min_idx; i--) {
            solution->path[i] = solution->path[i - 1];
        }

        solution->path[min_idx] = best_node;
        solution->cost += best_delta;
        visited[best_node] = 1;
        tour_size++;

    }

    solution->path[n] = solution->path[0];

    free(visited);

    if (run_2opt) {
        two_opt(solution, inst);
    }

    return 0;

}