#ifndef GREEDY_H
#define GREEDY_H

#include <stdlib.h>
#include <tsp_utilities.h>
#include <chrono.h>
#include <k_opt.h>

int greedy(int initial_point, tour* solution, int run_2opt, const instance* inst);
int greedy_multi_start(instance* inst, int run_2opt, double timelimit);

#endif // GREEDY_H