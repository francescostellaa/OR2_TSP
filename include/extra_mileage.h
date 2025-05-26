#ifndef EXTRA_MILEAGE_H
#define EXTRA_MILEAGE_H

#include <tsp_utilities.h>
#include <k_opt.h>  

int extra_mileage_multi_start(instance* inst, int run_2opt, double timelimit);
int extra_mileage(int start_node, tour* solution, int run_2opt, instance* inst);

#endif // EXTRA_MILEAGE_H