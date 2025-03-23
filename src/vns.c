#include <stdbool.h>
#include <vns.h>
/**
 * Implement the Variable Neighborhood Search (VNS) algorithm
 * @param inst
 * @return zero on success, non-zero on failures
 */
int vns(instance* inst) {

    int n = inst->nnodes;
    srand(inst->seed);
    greedy(rand() % n, inst, 0);

    int k = 1; 
    // k_max = maximum number of neighborhood changes
    // k = 1 is the 2-Opt neighborhood
    // k = 2 is the 3-Opt neighborhood
    // k = 3 is the 5-Opt neighborhood
    int k_max = 3;  

    tour* solution = malloc(sizeof(tour));
    solution->path = (int*)malloc((n+1) * sizeof(int));
    memcpy(solution->path, inst->best_sol->path, (n+1) * sizeof(int));
    solution->cost = inst->best_sol->cost;
    double prev_cost = inst->best_sol->cost;

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

                for (int i = 0; i < 5; i++) {
                    do {
                        indices_to_kick[0] = rand() % (n - 3);
                        indices_to_kick[1] = indices_to_kick[0] + 1 + (rand() % (n - indices_to_kick[0] - 3));
                        indices_to_kick[2] = indices_to_kick[1] + 1 + (rand() % (n - indices_to_kick[1] - 2));
                    } while ((indices_to_kick[1] <= indices_to_kick[0])    || 
                                (indices_to_kick[2] <= indices_to_kick[1]) || 
                                (indices_to_kick[2] > n - 1));
    
                    shake_three_edges(solution, inst, indices_to_kick); // 3-opt neighborhood
                }
        
            }
            else if (k > 2) {

                do{
                    indices_to_kick[0] = rand() % (n - 5);
                    indices_to_kick[1] = indices_to_kick[0] + 1 + (rand() % (n - indices_to_kick[0] - 5));
                    indices_to_kick[2] = indices_to_kick[1] + 1 + (rand() % (n - indices_to_kick[1] - 4));
                    indices_to_kick[3] = indices_to_kick[2] + 1 + (rand() % (n - indices_to_kick[2] - 3));
                    indices_to_kick[4] = indices_to_kick[3] + 1 + (rand() % (n - indices_to_kick[3] - 2));
                } while((indices_to_kick[1] <= indices_to_kick[0])    || 
                            (indices_to_kick[2] <= indices_to_kick[1]) || 
                            (indices_to_kick[3] <= indices_to_kick[2]) ||
                            (indices_to_kick[4] <= indices_to_kick[3]) ||
                            (indices_to_kick[4] > n - 1));
                
                shake_five_edges(solution, inst, indices_to_kick); // 5-opt neighborhood
            }
            
            free(indices_to_kick);
        } 
        compute_solution_cost(solution, inst);
        save_history_cost(solution->cost);

        two_opt(solution, inst);
        compute_solution_cost(solution, inst);
        current_cost = solution->cost;

        if (current_cost + EPS_COST < prev_cost){
            k = 1;
        }
        else {
            k++;
            if (k > k_max){
                k = 1;
            }
        }

        save_history_incumbent(inst->best_sol->cost);
        prev_cost = current_cost;

    }

    plot_solution(inst, inst->best_sol->path);
    plot_incumbent();
    plot_history_cost();
    plot_incumbent_and_costs();
    if(VERBOSE >= 1) { printf("Best cost: %lf\n", inst->best_sol->cost); }

    free(solution->path);
    free(solution);

    return 0;
}