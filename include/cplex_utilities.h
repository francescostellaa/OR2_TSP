#ifndef CPLEX_UTILITIES_H
#define CPLEX_UTILITIES_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <tsp_utilities.h>
#include <cplex.h>

#define EPS 1e-5

void save_history_benders(double cost, double time, const char* filename);
void plot_cost_benders(const char *input_file, const char *output_file);
void plot_succ_path(const int *succ, instance *inst, const char *output_file);
void reconstruct_sol(tour* solution, int* succ, const instance* inst);
void check_degrees(instance* inst, const double* xstar);
double compute_delta_straight(int i, int j, int* succ, instance* inst);
double compute_delta_cross(int i, int j, int* succ, instance* inst);
void reverse_component(int *succ, int best_j, int current_node, int past_node);
void from_solution_to_succ(int* succ, tour* solution, const instance* inst);

#endif // CPLEX_UTILITIES_H
