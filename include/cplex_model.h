#ifndef CPLEX_MODEL_H
#define CPLEX_MODEL_H

#include <tsp_utilities.h>
#include <cplex.h>
#include <k_opt.h>

int TSPopt(instance *inst);
int xpos(int i, int j, instance *inst);
void build_model(instance *inst, CPXENVptr env, CPXLPptr lp);
void build_sol(const double *xstar, instance *inst, int *succ, int *comp, int *ncomp);
void add_sec(CPXENVptr env, CPXLPptr lp, int comp_index, int *comp, instance *inst);
void xstar_to_solution(const double *xstar, instance *inst, int *solution);
void plot_xstar_path(const double *xstar, instance *inst, const char *output_file);
void save_history_benders(double cost, double time, const char* filename);
void plot_cost_benders(const char *input_file, const char *output_file);
void patching_heuristic(int *succ, int *ncomp, int *comp, instance *inst);
int check_duplicate_successors(int* succ, int n);

#endif //CPLEX_MODEL_H
