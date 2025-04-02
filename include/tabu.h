#ifndef TABU_H
#define TABU_H

#include <tsp_utilities.h>
#include <greedy.h>
#include <stdbool.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))

int tabu(const instance* inst, tour* solution, double timelimit, int interval_tenure, float tenure_scaling);

#endif //TABU_H
