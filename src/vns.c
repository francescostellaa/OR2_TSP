#include <stdbool.h>
#include <vns.h>

/**
 * Implement the Variable Neighborhood Search (VNS) algorithm
 * @param inst
 * @return zero on success, non-zero on failures
 */
int vns(instance* inst) {
    inst->tstart = second();
    if (VERBOSE >= 1000) { printf("Time start: %lf\n", inst->tstart); }

    int n = inst->nnodes;
    greedy(rand() % n, inst, 0);

    int iteration = 0;
    int k = 1; 
    int k_max = 3;  

    int* temp_sol = (int*)malloc((n+1) * sizeof(int));
    memcpy(temp_sol, inst->best_sol, (n+1) * sizeof(int));
    double prev_cost = inst->best_cost;

    while (true){

        double current_cost = INF_COST;
        if (second() - inst->tstart > inst->timelimit) {
            if (VERBOSE >= 100) { printf("Time limit reached\n"); }
            break;
        }

        // Shaking (changing neighborhood)
        if (k > 1){
            int* indices_to_kick = (int*)malloc(5 * sizeof(int));

            if (k == 2) {
                indices_to_kick[0] = rand() % (n - 3);
                indices_to_kick[1] = indices_to_kick[0] + 1 + (rand() % (n - indices_to_kick[0] - 3));
                indices_to_kick[2] = indices_to_kick[1] + 1 + (rand() % (n - indices_to_kick[1] - 2));
                shake_three_edges(temp_sol, inst, indices_to_kick); // 3-opt neighborhood
            }
            else if (k > 2) {
                indices_to_kick[0] = rand() % (n - 5);
                indices_to_kick[1] = indices_to_kick[0] + 1 + (rand() % (n - indices_to_kick[0] - 5));
                indices_to_kick[2] = indices_to_kick[1] + 1 + (rand() % (n - indices_to_kick[1] - 4));
                indices_to_kick[3] = indices_to_kick[2] + 1 + (rand() % (n - indices_to_kick[2] - 3));
                indices_to_kick[4] = indices_to_kick[3] + 1 + (rand() % (n - indices_to_kick[3] - 2));
                shake_five_edges(temp_sol, inst, indices_to_kick); // 5-opt neighborhood
            }
            
            free(indices_to_kick);
        } 

        save_history_cost(compute_solution_cost(temp_sol, inst));

        two_opt(temp_sol, inst);
        memcpy(temp_sol, inst->best_sol, (n+1) * sizeof(int));
        current_cost = compute_solution_cost(temp_sol, inst);

        if (current_cost < prev_cost){
            k = 1;  
        }
        else {
            k++;
            if (k > k_max){
                k = 1;
            }
        }

        save_history_incumbent(inst->best_cost);
        prev_cost = current_cost;

    }

    plot_solution(inst, inst->best_sol);
    plot_incumbent();
    plot_history_cost();
    plot_incumbent_and_costs();
    if(VERBOSE >= 1) { printf("Best cost: %lf\n", inst->best_cost); }

    free(temp_sol);

    return 0;
}