#ifndef CPLEX_MODEL_H
#define CPLEX_MODEL_H

#include <tsp_utilities.h>
#include <cplex.h>
#include <k_opt.h>
#include <vns.h>
#include <mincut.h>

typedef struct {
    CPXCALLBACKCONTEXTptr context;
    int ecount;
    int* elist;
    int* comp;
    int ncomp;   
    instance* inst;
    double* xstar;
} pass_params;

int TSPopt(instance *inst, int alg);
int xpos(int i, int j, instance *inst);
void build_model(instance *inst, CPXENVptr env, CPXLPptr lp);
void build_sol(const double *xstar, instance *inst, int *succ, int *comp, int *ncomp);
void add_sec(int* nnz, double* rhs, int comp_index, int* index, double* value, char** cname, int *comp, instance *inst);
void save_history_benders(double cost, double time, const char* filename);
void plot_cost_benders(const char *input_file, const char *output_file);
void post_heuristic_solution(CPXCALLBACKCONTEXTptr context, instance *inst, int *succ, int ncomp, int *comp, double objval);
void patching_heuristic(int *succ, int ncomp, int *comp, instance *inst);
int warm_start(CPXENVptr env, CPXLPptr lp, int* succ, instance *inst);
int benders(CPXENVptr env, CPXLPptr lp, instance *inst, int *succ, int ncomp, tour* solution);
int branch_and_cut(CPXENVptr env, CPXLPptr lp, CPXLONG contextid, instance *inst, int *succ, int ncomp, tour* solution);
int CPXPUBLIC candidate_callback(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle);
void solver(CPXENVptr env, CPXLPptr lp, instance *inst, double *xstar, int *succ, int *comp, int *ncomp, double *objval);
int CPXPUBLIC sec_callback(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void* userhandle);
int violated_cut_callback(double cut_val, int cut_nodes, int* cut, void* params);
double cut_violation(int nnz, double rhs, char sense, int* index, double* value, double* xstar);

#endif //CPLEX_MODEL_H
