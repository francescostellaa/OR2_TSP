#ifndef TABU_H
#define TABU_H

#include <tsp_utilities.h>
#include <greedy.h>
#include <stdbool.h>


int tabu(const instance* inst, tour* solution, double timelimit, int interval_tenure);
int tabu_multiparam(instance* inst, tour* solution, double timelimit, int* interval_tenure_params, int interval_tenure_size, char* output_file);


#endif //TABU_H
