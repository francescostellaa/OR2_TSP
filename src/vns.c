#include <stdbool.h>
#include <vns.h>
/**
 * Implement the Variable Neighborhood Search (VNS) algorithm
 * @param inst
 * @param solution
 * @param timelimit
 * @return zero on success, non-zero on failures
 */
int vns(const instance* inst, tour* solution, double timelimit) {

    int n = inst->nnodes;

    int k = 1; 
    // k_max = maximum number of neighborhood changes
    // k = 1 is the 2-Opt neighborhood
    // k = 2 is the 3-Opt neighborhood
    // k = 3 is the 5-Opt neighborhood
    int k_max = 3;  

    tour* best_solution = malloc(sizeof(tour));
    best_solution->path = (int*)malloc((n + 1) * sizeof(int));
    memcpy(best_solution->path, solution->path, (n + 1) * sizeof(int));
    double current_best_cost = solution->cost;

    while (true){

        double current_cost = INF_COST;
        if (second() - inst->tstart > timelimit) {
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
        save_history_cost(solution->cost, "../data/VNS/cost_vns.txt");

        two_opt(solution, inst);
        current_cost = solution->cost;
        if (current_cost + EPS_COST < current_best_cost){
            memcpy(best_solution->path, solution->path, (n + 1) * sizeof(int));
            current_best_cost = current_cost;
        }
        if (current_cost + EPS_COST < current_best_cost){
            k = 1;
        }
        else {
            k++;
            if (k > k_max){
                k = 1;
            }
        }

        save_history_incumbent(current_best_cost, "../data/VNS/incumbent_vns.txt");

    }

    memcpy(solution->path, best_solution->path, (n + 1) * sizeof(int));
    solution->cost = current_best_cost;

    free(best_solution->path);
    free(best_solution);
    plot_solution(inst, solution->path);
    plot_incumbent("../data/VNS/incumbent_vns.txt", "../data/VNS/incumbent_vns.png");
    plot_history_cost("../data/VNS/cost_vns.txt", "../data/VNS/cost_vns.png");
    plot_incumbent_and_costs("../data/VNS/cost_vns.txt", "../data/VNS/incumbent_vns.txt", "../data/VNS/incumbent_and_costs_vns.png");

    return 0;
}