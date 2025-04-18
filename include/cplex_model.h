#ifndef CPLEX_MODEL_H
#define CPLEX_MODEL_H

#include <tsp_utilities.h>
#include <cplex.h>
#include <k_opt.h>
#include <vns.h>

int TSPopt(instance *inst, int alg, int mode);
int xpos(int i, int j, instance *inst);
void build_model(instance *inst, CPXENVptr env, CPXLPptr lp);
void build_sol(const double *xstar, instance *inst, int *succ, int *comp, int *ncomp);
void add_sec(int* nnz, double* rhs, int comp_index, int* index, double* value, char** cname, int *comp, instance *inst);
void xstar_to_solution(const double *xstar, instance *inst, int *solution);
void plot_xstar_path(const double *xstar, instance *inst, const char *output_file);
void save_history_benders(double cost, double time, const char* filename);
void plot_cost_benders(const char *input_file, const char *output_file);
void patching_heuristic(int *succ, int ncomp, int *comp, instance *inst);
int check_duplicate_successors(int* succ, int n);
int warm_start(CPXENVptr env, CPXLPptr lp, int* succ, instance *inst);
int benders(CPXENVptr env, CPXLPptr lp, instance *inst, int *succ, int ncomp, tour* solution, int mode);
int branch_and_cut(CPXENVptr env, CPXLPptr lp, CPXLONG contextid, instance *inst, int *succ, int ncomp, tour* solution, int mode);
static int CPXPUBLIC my_callback(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle);
void solver(CPXENVptr env, CPXLPptr lp, instance *inst, double *xstar, int *succ, int *comp, int *ncomp, double *objval, int mode);

#endif //CPLEX_MODEL_H
