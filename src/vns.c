#include <stdbool.h>
#include <vns.h>
/**
 * Implement the Variable Neighborhood Search (VNS) algorithm
 * @param inst
 * @param solution
 * @param timelimit
 * @return zero on success, non-zero on failures
 */
int vns(const instance* inst, tour* solution, double timelimit, int num_kicks) {

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
            if (VERBOSE >= 100) { printf("Time limit of VNS reached\n"); }
            break;
        }

        // Shaking (changing neighborhood)
        if (k > 1){
            int* indices_to_kick = (int*)malloc(5 * sizeof(int));

            if (k == 2) {

                for (int i = 0; i < num_kicks; i++) {
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


int vns_multiparam(instance* inst, tour* solution, double timelimit, int* num_kicks_params, int num_kicks_size, char* output_file) {

    if (inst == NULL) {
        print_error("Error in initialization of the instance\n");
        return 1;
    }

    double* best_costs = (double*)malloc(sizeof(double) * num_kicks_size);

    for (int i = 0; i < num_kicks_size; i++){
        int k = num_kicks_params[i];

        tour* solution_copy = malloc(sizeof(tour));
        solution_copy->path = (int*)malloc((inst->nnodes + 1) * sizeof(int));
        memcpy(solution_copy->path, solution->path, (inst->nnodes + 1) * sizeof(int));
        solution_copy->cost = solution->cost;

        inst->tstart = second();

        if (vns(inst, solution_copy, timelimit, k)) {
            free(solution_copy->path);
            free(solution_copy);
            return 1;
        } 

        best_costs[i] = solution_copy->cost;

        free(solution_copy->path);
        free(solution_copy);
    }

    printf("Best costs for different number of kicks:\n");
    for (int i = 0; i < num_kicks_size; i++) {
        printf("num_kicks = %d: %.2f\n", num_kicks_params[i], best_costs[i]);
    }
    printf("\n");
    printf("Writing results to file: %s\n", output_file);

    char* output_line = (char*)malloc(256 * sizeof(char)); 
    output_line[0] = '\0'; 

    char seed_buffer[32];
    snprintf(seed_buffer, sizeof(seed_buffer), "inst%d;", inst->seed);
    strcat(output_line, seed_buffer);

    for (int i = 0; i < num_kicks_size; i++) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%.2f", best_costs[i]);
        strcat(output_line, buffer);
        if (i < num_kicks_size - 1) {
            strcat(output_line, ";");
        }
    }

    strcat(output_line, "\n");

    printf("%s\n", output_line);
    FILE *file = fopen(output_file, "a");

    if (file == NULL) {
        perror("Error opening file");
        return 1;
    }

    fprintf(file, "%s", output_line);
    fclose(file);
    
    free(best_costs);
    free(output_line);

    return 0;

}