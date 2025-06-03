#include "tabu.h"

/**
 * Implement the Tabu Search algorithm
 * @param inst instance containing the problem data
 * @param solution current solution to be improved
 * @param timelimit time limit for the algorithm
 * @param interval_tenure interval for updating the tenure
 * @param tenure_scaling scaling factor for the tenure
 * @return zero on success, non-zero on failures
 */
int tabu(const instance* inst, tour* solution, double timelimit, int interval_tenure, float tenure_scaling) {

    if (inst->best_sol->path == NULL) {
        print_error("Error occurred while allocating memory for the instance\n");
        return -1;
    }
    if (solution == NULL) {
        print_error("Error occurred while allocating memory for the solution\n");
        return -1;
    }
    if (solution->path == NULL) {
        print_error("Error occurred while allocating memory for the solution path\n");
        return -1;
    }

    int n = inst->nnodes;
    int tenure = inst->nnodes / 100 + 10;
    tour* best_solution = malloc(sizeof(tour));
    best_solution->path = (int*)malloc((n + 1) * sizeof(int));
    memcpy(best_solution->path, solution->path, (n + 1) * sizeof(int));
    double current_best_cost = solution->cost;

    int* when_tabu = malloc(sizeof(int) * n);
    if (when_tabu == NULL) {
        print_error("Error occurred while allocating memory for the tabu\n");
        return -1;
    }
    for (int i = 0; i < n; i++) {
        when_tabu[i] = -1;
    }
    int iteration = 0;

    while (true) {

        if (second() - inst->tstart > timelimit) {
            if (VERBOSE >= 100) { printf("Time limit reached\n"); }
            break;
        }

        double best_delta = INF_COST;
        int best_i = -1;
        int best_j = -1;

        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 1; j < n; j++) {
                bool is_tabu = ((when_tabu[solution->path[i]] + tenure > iteration ) ||
                               (when_tabu[solution->path[j]] + tenure > iteration ));

                if (!is_tabu) {
                    double delta = inst->cost_matrix[solution->path[i] * n + solution->path[j]] +
                                  inst->cost_matrix[solution->path[i + 1] * n + solution->path[j + 1]] -
                                  inst->cost_matrix[solution->path[i] * n + solution->path[i + 1]] -
                                  inst->cost_matrix[solution->path[j] * n + solution->path[j + 1]];
                    if (delta + EPS_COST < best_delta && delta != 0) {
                        best_delta = delta;
                        best_i = i;
                        best_j = j;
                    }
                }
            }
        }

        if (iteration % interval_tenure == 0) {
            tenure = (int)MIN(sqrt(inst->nnodes) + rand() % (int)(tenure_scaling * inst->nnodes + 1), inst->nnodes);
        }
        iteration++;
        if (best_i != -1 && best_j != -1) {
            if (best_delta + EPS_COST >= 0) {
                when_tabu[solution->path[best_i]] = iteration;
                when_tabu[solution->path[best_j]] = iteration;
            }
            reverse_segment(solution->path, best_i + 1, best_j);
            solution->cost += best_delta;
        }

        if (solution->cost + EPS_COST < current_best_cost){
            memcpy(best_solution->path, solution->path, (n + 1) * sizeof(int));
            current_best_cost = solution->cost;
        }

        save_history_incumbent(current_best_cost, "../data/TABU/incumbent_tabu.txt");
        save_history_cost(solution->cost, "../data/TABU/cost_tabu.txt");
    }

    plot_solution(inst, solution->path);
    plot_incumbent("../data/TABU/incumbent_tabu.txt", "../data/TABU/incumbent_tabu.png");
    plot_history_cost("../data/TABU/cost_tabu.txt", "../data/TABU/cost_tabu.png");
    plot_incumbent_and_costs("../data/TABU/cost_tabu.txt", "../data/TABU/incumbent_tabu.txt", "../data/TABU/incumbent_and_costs_tabu.png");
    free(when_tabu);

    memcpy(solution->path, best_solution->path, (n + 1) * sizeof(int));
    solution->cost = current_best_cost;
    free(best_solution->path);
    free(best_solution);
    return 0;
}
