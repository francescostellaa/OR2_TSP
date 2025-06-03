#include "k_opt.h"

/**
 * Apply the 2-opt refinement to the solution
 * @param solution tour to optimize
 * @param inst instance containing the problem data
 */
void two_opt(tour* solution, const instance* inst) {
    const int n = inst->nnodes;

    int improvement = 1;
    while (improvement) {
        if(second() - inst->tstart > inst->timelimit) {
            break;
        }

        // Initialize variables to find best swap
        improvement = 0;
        double best_delta = 0;
        int best_i = -1;
        int best_j = -1;
        // Iterate through all pairs of edges in the tour to find the best swap
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
}


/**
 * Shake the solution by swapping three edges, performing kicks in the solution
 * @param solution original tour to shake
 * @param inst instance containing the problem data
 * @param elements_to_swap array containing the indices of the edges to swap
 */
void shake_three_edges(tour* solution, const instance* inst, const int* elements_to_swap){

    const int i = elements_to_swap[0];
    const int j = elements_to_swap[1];
    const int k = elements_to_swap[2];

    const int reconnection = rand() % 4;

    // Perform the reconnection based on the random choice
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

    // Check solution feasibility
    compute_solution_cost(solution, inst);
    check_sol(solution->path, solution->cost, inst);
}


/**
 * Shake the solution by swapping five edges, performing kicks in the solution
 * @param solution original tour to shake
 * @param inst instance containing the problem data
 * @param elements_to_swap array containing the indices of the edges to swap
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

    // Check solution feasibility
    compute_solution_cost(solution, inst);
    check_sol(solution->path, solution->cost, inst);
}