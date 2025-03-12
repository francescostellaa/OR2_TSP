#include "k_opt.h"

/**
 * Apply the 2-opt refinement to the solution
 */
void two_opt(int* solution, instance* inst) {
    int* temp_solution = (int*)malloc((inst->nnodes + 1) * sizeof(int));
    memcpy(temp_solution, solution, (inst->nnodes + 1) * sizeof(int));
    double temp_cost = inst->best_cost;
    int n = inst->nnodes;

    int improvement = 1;

    while (improvement) {

        if(second() - inst->tstart > inst->timelimit) {
            if ( VERBOSE >= 100 ) { printf("Time limit reached\n"); }
            break;
        }

        improvement = 0;
        double best_delta = 0;
        int best_i = -1;
        int best_j = -1;

        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 1; j < n; j++) {
                double delta = inst->cost[temp_solution[i] * n + temp_solution[j]] +
                               inst->cost[temp_solution[i + 1] * n + temp_solution[j + 1]] -
                               inst->cost[temp_solution[i] * n + temp_solution[i + 1]] -
                               inst->cost[temp_solution[j] * n + temp_solution[j + 1]];

                if (delta < best_delta) {
                    best_delta = delta;
                    best_i = i;
                    best_j = j;
                }
            }
        }
                
        if (best_delta < -EPS_COST) {
            // Perform the 2-opt swap
            int i = best_i+1;
            int j = best_j;
            while (i < j) {
                swap(temp_solution, i, j);
                i++;
                j--;
            }
            temp_cost += best_delta;
            improvement = 1;
        }

    }

    update_best_sol(inst, temp_solution, temp_cost);

    free(temp_solution);
}


void reverse_segment(int* sol, int start, int end) {
    while (start < end) {
        int temp = sol[start];
        sol[start] = sol[end];
        sol[end] = temp;
        start++;
        end--;
    }
}

void shake_three_edges(int* solution, instance* inst, int* elements_to_swap){

    int* temp_solution = (int*)malloc((inst->nnodes + 1) * sizeof(int));
    memcpy(temp_solution, solution, (inst->nnodes + 1) * sizeof(int));
    double temp_cost = inst->best_cost;
    int n = inst->nnodes;

    int i = elements_to_swap[0];
    int j = elements_to_swap[1];
    int k = elements_to_swap[2];

    int reconnection = rand() % 4;

    switch (reconnection)
    {
        case 0:
            reverse_segment(temp_solution, j + 1, k);
            reverse_segment(temp_solution, i + 1, j);
            reverse_segment(temp_solution, j + 1, k);
            break;
        case 1:
            reverse_segment(temp_solution, i + 1, j);
            reverse_segment(temp_solution, j + 1, k);
            reverse_segment(temp_solution, i + 1, j);
            break;
        case 2:
            reverse_segment(temp_solution, j + 1, k);
            reverse_segment(temp_solution, i + 1, j);
            reverse_segment(temp_solution, j + 1, k);
            break;
        case 3:
            reverse_segment(temp_solution, i + 1, k);
            reverse_segment(temp_solution, j + 1, k);
            reverse_segment(temp_solution, i + 1, j);
            break;
    
    }

}