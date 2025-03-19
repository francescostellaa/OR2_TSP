#include "tabu.h"



int tabu(instance* inst) {
    int n = inst->nnodes;
    greedy(rand() % n, inst, 0);
    tour* solution = malloc(sizeof(tour));
    solution->path = (int*)malloc((n+1) * sizeof(int));
    memcpy(solution->path, inst->best_sol->path, (n+1) * sizeof(int));
    solution->cost = inst->best_sol->cost;

    tabu_move* tabu_list = malloc(TENURE * sizeof(tabu_move));
    int tabu_list_size = 0;
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
                bool is_tabu = (when_tabu[solution->path[i]] > iteration - TENURE) ||
                               (when_tabu[solution->path[j]] > iteration - TENURE);

                if (!is_tabu) {
                    double delta = inst->cost_matrix[solution->path[i] * n + solution->path[j]] +
                                  inst->cost_matrix[solution->path[i + 1] * n + solution->path[j + 1]] -
                                  inst->cost_matrix[solution->path[i] * n + solution->path[i + 1]] -
                                  inst->cost_matrix[solution->path[j] * n + solution->path[j + 1]];
                    if (delta + EPS_COST < best_delta) {
                        best_delta = delta;
                        best_i = i;
                        best_j = j;
                    }
                }
            }
        }

        when_tabu[best_i] = iteration;
        when_tabu[best_j] = iteration;
        //tabu_move move = {best_i, best_j};
        //tabu_list[tabu_list_size % TENURE] = move;
        //tabu_list_size++;
        reverse_segment(solution->path, best_i + 1, best_j);
        solution->cost += best_delta;
        update_best_sol(inst, solution);


        iteration++;
    }

    plot_solution(inst, inst->best_sol->path);

    free(tabu_list);
    free(when_tabu);
    free(solution);
    if(VERBOSE >= 1) { printf("Best cost: %lf\n", inst->best_sol->cost); }
    return 0;
}
