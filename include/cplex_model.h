#ifndef CPLEX_MODEL_H
#define CPLEX_MODEL_H

#include <tsp_utilities.h>
#include <cplex.h>

int TSPopt(instance *inst);
int xpos(int i, int j, instance *inst);
void build_model(instance *inst, CPXENVptr env, CPXLPptr lp);
void build_sol(const double *xstar, instance *inst, int *succ, int *comp, int *ncomp);
double dist_cplex(int i, int j, instance *inst);
void xstar_to_solution(const double *xstar, instance *inst, int *solution);
void plot_xstar_solution(const double *xstar, instance *inst);

#endif //CPLEX_MODEL_H
