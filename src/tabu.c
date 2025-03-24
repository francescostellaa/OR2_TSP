#include "tabu.h"

int tabu(instance* inst) {
    int n = inst->nnodes;
    srand(inst->seed);
    greedy(rand() % n, inst, 0);
    tour* solution = malloc(sizeof(tour));
    solution->path = (int*)malloc((n+1) * sizeof(int));
    memcpy(solution->path, inst->best_sol->path, (n+1) * sizeof(int));
    solution->cost = inst->best_sol->cost;
    int TENURE = inst->nnodes / 100 + 10;

    int* when_tabu = malloc(sizeof(int) * n);
    for (int i = 0; i < n; i++) {
        when_tabu[i] = -1;
    }
    printf("Best cost: %lf\n", inst->best_sol->cost);
    int iteration = 0;

    while (true) {

        if (second() - inst->tstart > inst->timelimit) {
            if (VERBOSE >= 100) { printf("Time limit reached\n"); }
            break;
        }

        double best_delta = INF_COST;
        int best_i = -1;
        int best_j = -1;

        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 1; j < n; j++) {
                bool is_tabu = ((when_tabu[solution->path[i]] + TENURE > iteration ) ||
                               (when_tabu[solution->path[j]] + TENURE > iteration ));

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

        if (iteration % 100 == 0) {
            TENURE = (int)(sqrt(inst->nnodes) + rand() % (int)(inst->nnodes / sqrt(inst->nnodes) + 1));
        }
        iteration++;
        if (best_i != -1 && best_j != -1) {
            when_tabu[solution->path[best_i]] = iteration;
            when_tabu[solution->path[best_j]] = iteration;
            reverse_segment(solution->path, best_i + 1, best_j);
            solution->cost += best_delta;
        }

        update_best_sol(inst, solution);
        save_history_incumbent(inst->best_sol->cost);
        save_history_cost(solution->cost);
    }

    plot_solution(inst, inst->best_sol->path);
    plot_incumbent();
    plot_history_cost();
    plot_incumbent_and_costs();
    free(when_tabu);
    free(solution->path);
    free(solution);
    if(VERBOSE >= 1) { printf("Best cost: %lf\n", inst->best_sol->cost); }
    return 0;
}