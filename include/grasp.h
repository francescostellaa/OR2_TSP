#ifndef GRASP_H
#define GRASP_H

#include <tsp_utilities.h>
#include <k_opt.h>

int grasp(int initial_point, instance* inst);
int grasp_multi_start(instance* inst);

#endif //GRASP_H
