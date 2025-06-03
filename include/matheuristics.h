#ifndef MATHEURISTICS_H
#define MATHEURISTICS_H

#include <greedy.h>
#include <vns.h>
#include <cplex_model.h>
#include <cplex_utilities.h>
#include <tsp_utilities.h>

double min(double a, double b);
int hard_fixing(instance* inst, parameters* parameters);
int local_branching(instance* inst, parameters* parameters);
void initialize_cplex(instance* inst, CPXENVptr* env, CPXLPptr* lp);

#endif //MATHEURISTICS_H
