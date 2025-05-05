#include "k_opt.h"

/**
 * Apply the 2-opt refinement to the solution
 */
void two_opt(tour* solution, const instance* inst) {
    const int n = inst->nnodes;

    int improvement = 1;
    while (improvement) {
        if(second() - inst->tstart > inst->timelimit) {
            break;
        }

        improvement = 0;
        double best_delta = 0;
        int best_i = -1;
        int best_j = -1;

        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 1; j < n; j++) {
                double delta = inst->cost_matrix[solution->path[i] * n + solution->path[j]] +
                               inst->cost_matrix[solution->path[i + 1] * n + solution->path[j + 1]] -
                               inst->cost_matrix[solution->path[i] * n + solution->path[i + 1]] -
                               inst->cost_matrix[solution->path[j] * n + solution->path[j + 1]];
                if (delta + EPS_COST < best_delta) {
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
                swap(solution->path, i, j);
                i++;
                j--;
            }
            solution->cost += best_delta;
            improvement = 1;
        }
    }
    //update_best_sol(inst, solution);
}


/*
void two_opt(tour* solution, const instance* inst) {
    const int n = inst->nnodes;

    // Pre-allocate arrays to avoid repeated swaps
    int* new_path = (int*)malloc(n * sizeof(int));

    int improvement = 1;
    while (improvement) {
        // Check time limit early
        if(second() - inst->tstart > inst->timelimit) {
            break;
        }

        improvement = 0;
        double best_delta = 0;
        int best_i = -1;
        int best_j = -1;

        // Don't recalculate the entire tour cost for each potential swap
        // Instead, just calculate the delta (change in cost)
        for (int i = 0; i < n - 2; i++) {
            int i_node = solution->path[i];
            int i_next = solution->path[i + 1];
            double cost_i_inext = inst->cost_matrix[i_node * n + i_next];

            for (int j = i + 2; j < n; j++) {
                int j_node = solution->path[j];
                int j_next = solution->path[(j + 1) % n]; // Handle wrap-around for last node

                // Calculate cost difference for this 2-opt swap
                double delta = inst->cost_matrix[i_node * n + j_node] +
                               inst->cost_matrix[i_next * n + j_next] -
                               cost_i_inext -
                               inst->cost_matrix[j_node * n + j_next];

                if (delta + EPS_COST < best_delta) {
                    best_delta = delta;
                    best_i = i;
                    best_j = j;
                }
            }
        }

        if (best_delta + EPS_COST < 0) {
            // Use a more efficient approach for the 2-opt swap
            // First, copy the unchanged part before the swap
            for (int k = 0; k <= best_i; k++) {
                new_path[k] = solution->path[k];
            }

            // Reverse the segment between i+1 and j
            for (int k = best_i + 1, l = best_j; k <= best_j; k++, l--) {
                new_path[k] = solution->path[l];
            }

            // Copy the unchanged part after the swap
            for (int k = best_j + 1; k < n; k++) {
                new_path[k] = solution->path[k];
            }

            // Update the solution path
            for (int k = best_i + 1; k <= best_j; k++) {
                solution->path[k] = new_path[k];
            }

            solution->cost += best_delta;
            improvement = 1;
        }
    }

    free(new_path);
}
*/


/**
 * Shake the solution by swapping three edges, performing kicks in the solution
 */
void shake_three_edges(tour* solution, const instance* inst, const int* elements_to_swap){

    const int i = elements_to_swap[0];
    const int j = elements_to_swap[1];
    const int k = elements_to_swap[2];

    const int reconnection = rand() % 4;

    switch (reconnection)
    {
        case 0:
            reverse_segment(solution->path, i+1, k);
            reverse_segment(solution->path, j+1, k);
            break;
        case 1:
            reverse_segment(solution->path, j+1, k+1);
            reverse_segment(solution->path, i+1, k+1);
            reverse_segment(solution->path, j+1, k+1);
            break;
        case 2:
            reverse_segment(solution->path, i+1, k);
            reverse_segment(solution->path, i+1, j);
            break;
        case 3:
            reverse_segment(solution->path, i+1, j);
            reverse_segment(solution->path, j+1, k);
            break;
        default:
            break;
    }

    compute_solution_cost(solution, inst);
    check_sol(solution->path, solution->cost, inst);
}


/**
 * Shake the solution by swapping five edges, performing kicks in the solution
 */
void shake_five_edges(tour* solution, const instance* inst, const int* elements_to_swap){

    const int i = elements_to_swap[0];
    const int j = elements_to_swap[1];
    const int k = elements_to_swap[2];
    const int l = elements_to_swap[3];
    const int m = elements_to_swap[4];
    
    const int reconnection = rand() % 3;

    switch (reconnection)
    {
        case 0:
            reverse_segment(solution->path, j+1, m);
            reverse_segment(solution->path, i+1, l);
            reverse_segment(solution->path, i+1, j);
            reverse_segment(solution->path, j+1, l);
            reverse_segment(solution->path, k+1, l);
            break;
        case 1:
            reverse_segment(solution->path, i+1, k);
            reverse_segment(solution->path, i+1, j);
            reverse_segment(solution->path, j+1, k);
            reverse_segment(solution->path, k+1, m);
            reverse_segment(solution->path, k+1, l);
            break;
        case 2:
            reverse_segment(solution->path, i+1, k);
            reverse_segment(solution->path, j+1, m);
            reverse_segment(solution->path, j+1, k);
            break;
        default:
            break;
    }

    compute_solution_cost(solution, inst);
    check_sol(solution->path, solution->cost, inst);
}