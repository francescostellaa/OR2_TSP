#include <vns.h>

int vns(instance* inst) {
    inst->tstart = second();
    if (VERBOSE >= 1000) { printf("Time start: %lf\n", inst->tstart); }
    greedy(0, inst, 0);

    int max_iterations = 100;
    int iteration = 0;
    int k = 2; // 2-opt neighborhood
    int k_max = 3;  // N-opt neighborhood
    int n = inst->nnodes;
    inst->best_cost_history = (double*)malloc(max_iterations * sizeof(double));

    while (iteration < max_iterations){
        iteration++;

        if (k > 2){
            int* indices_to_kick = (int*)malloc(k_max * sizeof(int));
            indices_to_kick[0] = rand() % (n - 2);
            indices_to_kick[1] = indices_to_kick[0] + 1 + (rand() % (n - indices_to_kick[0] - 1));
            indices_to_kick[2] = indices_to_kick[1] + 1 + (rand() % (n - indices_to_kick[1] - 1));
            for (int i = 3; i < k_max; i++){
                indices_to_kick[i] = rand() % n;
            }
            shake(inst->best_sol, inst, indices_to_kick);

            free(indices_to_kick);
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

    return 0;
}