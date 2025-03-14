#include <vns.h>

int vns(instance* inst) {
    inst->tstart = second();
    if (VERBOSE >= 1000) { printf("Time start: %lf\n", inst->tstart); }

    int n = inst->nnodes;
    greedy(rand() % n, inst, 0);

    int iteration = 0;
    int k = 2; // 2-opt neighborhood
    int k_max = 3;  // N-opt neighborhood

    int* temp_sol = (int*)malloc((n+1) * sizeof(int));
    memcpy(temp_sol, inst->best_sol, (n+1) * sizeof(int));
    double prev_cost = inst->best_cost;

    while (iteration < MAX_ITERATIONS_VNS){

        if (second() - inst->tstart > inst->timelimit) {
            if (VERBOSE >= 100) { printf("Time limit reached\n"); }
            break;
        }

        iteration++;

        // Shaking (changing neighborhood)
        if (k > 2){
            int* indices_to_kick = (int*)malloc(k_max * sizeof(int));

            if (k == 3) {
                indices_to_kick[0] = rand() % (n - 3);
                indices_to_kick[1] = indices_to_kick[0] + 1 + (rand() % (n - indices_to_kick[0] - 3));
                indices_to_kick[2] = indices_to_kick[1] + 1 + (rand() % (n - indices_to_kick[1] - 2));

                shake_three_edges(temp_sol, inst, indices_to_kick);
            } 
            // else if (k > 3) {
            //     // Implement other neighborhoods here
            // }
            
            free(indices_to_kick);
        } else {
            // Local search
            two_opt(temp_sol, inst);
        }

        double current_cost = compute_solution_cost(temp_sol, inst);
        
        if (current_cost < prev_cost){
            k = 2;  
        }
        else {
            k++;
            if (k > k_max){
                k = 2;
            }
        }
        prev_cost = current_cost;

    }

    plot_solution(inst, inst->best_sol);
    plot_incumbent();
    if(VERBOSE >= 1) { printf("Best cost: %lf\n", inst->best_cost); }

    free(temp_sol);

    return 0;
}