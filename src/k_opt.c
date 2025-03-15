#include "k_opt.h"

/**
 * Apply the 2-opt refinement to the solution
 */
void two_opt(int* solution, instance* inst) {
    int* temp_solution = (int*)malloc((inst->nnodes + 1) * sizeof(int));
    memcpy(temp_solution, solution, (inst->nnodes + 1) * sizeof(int));
    double temp_cost = compute_solution_cost(temp_solution, inst);
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

        if (best_delta + EPS_COST < 0) {
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

/**
 * Reverse the segment between start and end in the solution
 */
void reverse_segment(int* sol, int start, int end) {
    while (start < end) {
        swap(sol, start, end);
        start++;
        end--;
    }
}

/**
 * Shake the solution by swapping three edges, performing kicks in the solution
 */
void shake_three_edges(int* solution, instance* inst, int* elements_to_swap){

    int* temp_solution = (int*)malloc((inst->nnodes + 1) * sizeof(int));
    memcpy(temp_solution, solution, (inst->nnodes + 1) * sizeof(int));
    //int n = inst->nnodes;

    int i = elements_to_swap[0];
    int j = elements_to_swap[1];
    int k = elements_to_swap[2];

    int reconnection = rand() % 4;

    switch (reconnection)
    {
        case 0:
            reverse_segment(temp_solution, i+1, k);
            reverse_segment(temp_solution, j+1, k);
            break;
        case 1:
            reverse_segment(temp_solution, j+1, k+1);
            reverse_segment(temp_solution, i+1, k+1);
            reverse_segment(temp_solution, j+1, k+1);
            break;
        case 2:
            reverse_segment(temp_solution, i+1, k);
            reverse_segment(temp_solution, i+1, j);
            break;
        case 3:
            reverse_segment(temp_solution, i+1, j);
            reverse_segment(temp_solution, j+1, k);
            break;
        default:
            break;
    }

    double temp_cost = compute_solution_cost(temp_solution, inst);
    if (check_sol(temp_solution, temp_cost, inst)){
        for (int i = 0; i < inst->nnodes + 1; i++) {
            solution[i] = temp_solution[i];
        }
    }
    free(temp_solution);
}


/**
 * Shake the solution by swapping five edges, performing kicks in the solution
 */
void shake_five_edges(int* solution, instance* inst, int* elements_to_swap){

    int* temp_solution = (int*)malloc((inst->nnodes + 1) * sizeof(int));
    memcpy(temp_solution, solution, (inst->nnodes + 1) * sizeof(int));

    int i = elements_to_swap[0];
    int j = elements_to_swap[1];
    int k = elements_to_swap[2];
    int l = elements_to_swap[3];
    int m = elements_to_swap[4];
    
    int reconnection = rand() % 3;

    switch (reconnection)
    {
        case 0:
            reverse_segment(temp_solution, j+1, m);
            reverse_segment(temp_solution, i+1, l);
            reverse_segment(temp_solution, i+1, j);
            reverse_segment(temp_solution, j+1, l);
            reverse_segment(temp_solution, k+1, l);
            break;
        case 1:
            reverse_segment(temp_solution, i+1, k);
            reverse_segment(temp_solution, i+1, j);
            reverse_segment(temp_solution, j+1, k);
            reverse_segment(temp_solution, k+1, m);
            reverse_segment(temp_solution, k+1, l);
            break;
        case 2:
            reverse_segment(temp_solution, i+1, k);
            reverse_segment(temp_solution, j+1, m);
            reverse_segment(temp_solution, j+1, k);
            break;
        default:
            break;
    }

    double temp_cost = compute_solution_cost(temp_solution, inst);
    if (check_sol(temp_solution, temp_cost, inst)){
        for (int i = 0; i < inst->nnodes + 1; i++) {
            solution[i] = temp_solution[i];
        }

    }
    free(temp_solution);
}