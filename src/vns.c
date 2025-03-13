#include <vns.h>

int vns(instance* inst) {
    inst->tstart = second();
    if (VERBOSE >= 1000) { printf("Time start: %lf\n", inst->tstart); }
    greedy(0, inst, 0);

    int max_iterations = 5000;
    int iteration = 0;
    int k = 2; // 2-opt neighborhood
    int k_max = 3;  // N-opt neighborhood
    int n = inst->nnodes;
    inst->best_cost_history = (double*)malloc(max_iterations * sizeof(double));

    while (iteration < max_iterations){
        iteration++;

        if (k > 2){
            for (int i = 0; i < 3; i++){
                int* indices_to_kick = (int*)malloc(k_max * sizeof(int));
                indices_to_kick[0] = rand() % (n - 2);
                indices_to_kick[1] = indices_to_kick[0] + 1 + (rand() % (n - indices_to_kick[0] - 2));
                indices_to_kick[2] = indices_to_kick[1] + 1 + (rand() % (n - indices_to_kick[1] - 1));

                shake_three_edges(inst->best_sol, inst, indices_to_kick);

                free(indices_to_kick);
            }
        }

        two_opt(inst->best_sol, inst);
        

        if (inst->history_size == 0 || inst->best_cost < inst->best_cost_history[inst->history_size - 1]){
            inst->best_cost_history[inst->history_size++] = inst->best_cost;
            k = 2;
        }
        else {
            k++;
            if (k > k_max){
                k = 2;
            }
        }

    }

    plot_solution(inst, inst->best_sol);
    if(VERBOSE >= 1) { printf("Best cost: %lf\n", inst->best_cost); }

    return 0;
}