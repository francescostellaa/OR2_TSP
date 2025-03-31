#include "tabu.h"

int tabu(const instance* inst, tour* solution, double timelimit, int interval_tenure) {
    printf("Interval Tenure = %d\n", interval_tenure);
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
            tenure = (int)(sqrt(inst->nnodes) + rand() % (int)(inst->nnodes / sqrt(inst->nnodes) + 1));
        }
        iteration++;
        if (best_i != -1 && best_j != -1) {
            when_tabu[solution->path[best_i]] = iteration;
            when_tabu[solution->path[best_j]] = iteration;
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


int tabu_multiparam(instance* inst, tour* solution, double timelimit, int* interval_tenure_params, int interval_tenure_size, char* output_file) {

    if (inst == NULL) {
        print_error("Error in initialization of the instance\n");
        return 1;
    }

    double* best_costs = (double*)malloc(sizeof(double) * interval_tenure_size);

    for (int i = 0; i < interval_tenure_size; i++){
        int k = interval_tenure_params[i];

        tour* solution_copy = malloc(sizeof(tour));
        solution_copy->path = (int*)malloc((inst->nnodes + 1) * sizeof(int));
        memcpy(solution_copy->path, solution->path, (inst->nnodes + 1) * sizeof(int));
        solution_copy->cost = solution->cost;

        inst->tstart = second();

        if (tabu(inst, solution_copy, timelimit, k)) {
            free(solution_copy->path);
            free(solution_copy);
            return 1;
        } 

        best_costs[i] = solution_copy->cost;

        free(solution_copy->path);
        free(solution_copy);
    }

    char* output_line = (char*)malloc(256 * sizeof(char)); 
    output_line[0] = '\0'; 

    char seed_buffer[32];
    snprintf(seed_buffer, sizeof(seed_buffer), "inst%d;", inst->seed);
    strcat(output_line, seed_buffer);

    for (int i = 0; i < interval_tenure_size; i++) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%.2f", best_costs[i]);
        strcat(output_line, buffer);
        if (i < interval_tenure_size - 1) {
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