#ifndef GRASP_H
#define GRASP_H

#include <tsp_utilities.h>
#include <k_opt.h>

int grasp(int initial_point, const instance* inst, tour* solution);
int grasp_multi_start(instance* inst, double timelimit);

#endif //GRASP_H
