#ifndef GRASP_H
#define GRASP_H

#include <tsp_utilities.h>
#include <k_opt.h>

int grasp(int initial_point, const instance* inst, tour* solution, double prob_grasp);
int grasp_multi_start(instance* inst, double timelimit, double prob_grasp);

#endif //GRASP_H
