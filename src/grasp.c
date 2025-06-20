#include <grasp.h>

/**
 * Implement the Greedy Randomized Adaptive Search Procedure (GRASP) algorithm
 * @param initial_point the index of the initial point to start the tour
 * @param inst instance containing the problem data
 * @param solution solution tour to be constructed
 * @param prob_grasp probability of not choosing the nearest neighbor
 * @return zero on success, non-zero on failures
 */
int grasp(int initial_point, const instance* inst, tour* solution, double prob_grasp) {

    //Check correct memory allocation
    if (solution == NULL) { print_error("Error in Grasp\n"); }
    if (solution->path == NULL) { print_error("Error in Grasp\n"); }

    // Initialize the solution with node indices
    for (int i = 0; i < inst->nnodes; i++) {
        solution->path[i] = i;
    }
    swap(solution->path, 0, initial_point); // Start from the initial point
    int n = inst->nnodes;

    // Constructive Phase
    for (int i = 0; i < inst->nnodes - 1; i++) {
        // Arrays to store the nearest neighbor costs and indices
        double min_costs[4] = {INF_COST, INF_COST, INF_COST, INF_COST};
        int min_indices[4] = {-1, -1, -1, -1};

        // Find the nearest, second, third, and fourth nearest neighbors
        for (int j = i + 1; j < inst->nnodes; j++) {
            double current_cost = inst->cost_matrix[solution->path[i] * inst->nnodes + solution->path[j]];

            // Update the nearest, second, third, and fourth nearest neighbors
            if (current_cost < min_costs[0]) {
                // Shift existing values down
                min_costs[3] = min_costs[2];
                min_costs[2] = min_costs[1];
                min_costs[1] = min_costs[0];
                min_costs[0] = current_cost;

                min_indices[3] = min_indices[2];
                min_indices[2] = min_indices[1];
                min_indices[1] = min_indices[0];
                min_indices[0] = j;
            } else if (current_cost < min_costs[1]) {
                min_costs[3] = min_costs[2];
                min_costs[2] = min_costs[1];
                min_costs[1] = current_cost;

                min_indices[3] = min_indices[2];
                min_indices[2] = min_indices[1];
                min_indices[1] = j;
            } else if (current_cost < min_costs[2]) {
                min_costs[3] = min_costs[2];
                min_costs[2] = current_cost;

                min_indices[3] = min_indices[2];
                min_indices[2] = j;
            } else if (current_cost < min_costs[3]) {
                min_costs[3] = current_cost;
                min_indices[3] = j;
            }
        }

        // Decide whether to choose the nearest neighbor or a random alternative
        int selected_index;

        //if (rand() % 100 < 95 || min_costs[1] == INF_COST || min_costs[2] == INF_COST || min_costs[3] == INF_COST) {
        if (random01() > prob_grasp || min_costs[1] == INF_COST || min_costs[2] == INF_COST || min_costs[3] == INF_COST) {
            // 95% chance: choose the nearest neighbor
            selected_index = min_indices[0];
            solution->cost += min_costs[0];
        } else {
            // 5% chance: randomly choose among the second, third, or fourth nearest neighbors
            int random_choice = rand() % 3; // 0, 1, or 2
            selected_index = min_indices[random_choice + 1];
            solution->cost += min_costs[random_choice + 1];
        }

        // Add the selected node to the solution
        swap(solution->path, i + 1, selected_index);
    }

    // Complete the tour by returning to the starting node
    solution->path[n] = solution->path[0];
    solution->cost += inst->cost_matrix[solution->path[n - 1] * n + solution->path[n]];

    // Save the cost of the current solution
    save_history_cost(solution->cost, "../data/cost_grasp.txt");

    // Local Search Phase (2-opt heuristic)
    two_opt(solution, inst);

    // Save the cost of the current solution
    save_history_cost(solution->cost, "../data/cost_grasp.txt");

    return 0;
}

/**
 * Multi-start GRASP algorithm to solve the TSP
 * @param inst instance
 * @param timelimit time limit for the algorithm
 * @param prob_grasp probability of not choosing the nearest neighbor
 * @return zero on success, non-zero on failures
 */
int grasp_multi_start(instance* inst, double timelimit, double prob_grasp) {

    if (inst == NULL) {
        print_error("Error in initialization of the instance\n");
        return -1;
    }
    if (inst->best_sol == NULL) {
        printf("No solution exists in the instance\n");
        return -1;
    }
    if (inst->best_sol->path == NULL) {
        print_error("Error occurred while allocating memory for the instance solution\n");
        return -1;
    }

    const int n = inst->nnodes;

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
        }//245654.511435

        if (grasp(i, inst, solution, prob_grasp) == 0) {
            update_best_sol(inst, solution);
        }
        save_history_incumbent(inst->best_sol->cost, "../data/incumbent_grasp.txt");
        free(solution->path);
        free(solution);
    }

    plot_solution(inst, inst->best_sol->path);
    plot_incumbent("../data/incumbent_grasp.txt", "../data/incumbent_grasp.png");
    plot_history_cost("../data/cost_grasp.txt", "../data/cost_grasp.png");
    if(VERBOSE >= 1) { printf("Best cost: %lf\n", inst->best_sol->cost); }

    return 0;
}