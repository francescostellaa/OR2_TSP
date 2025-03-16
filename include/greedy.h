#ifndef GREEDY_H
#define GREEDY_H

#include <stdlib.h>
#include <tsp_utilities.h>
#include <chrono.h>
#include <k_opt.h>

int greedy(int initial_point, instance* inst, int run_2opt);
int greedy_multi_start(instance* inst, int run_2opt);

#endif // GREEDY_H