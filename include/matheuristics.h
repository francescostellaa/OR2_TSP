#ifndef MATHEURISTICS_H
#define MATHEURISTICS_H

#include <greedy.h>
#include <vns.h>
#include <cplex_model.h>
#include <cplex_utilities.h>
#include <tsp_utilities.h>

double min(double a, double b);
int hard_fixing(instance* inst);

#endif //MATHEURISTICS_H
