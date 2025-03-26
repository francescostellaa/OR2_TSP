#ifndef K_OPT_H
#define K_OPT_H

#include <stdlib.h>
#include <tsp_utilities.h>
#include <chrono.h>

void two_opt(tour* solution, const instance* inst);
void shake_three_edges(tour* solution, const instance* inst, const int* elements_to_swap);
void shake_five_edges(tour* solution, const instance* inst, const int* elements_to_swap);

#endif // K_OPT