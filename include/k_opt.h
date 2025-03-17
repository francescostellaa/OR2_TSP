#ifndef K_OPT_H
#define K_OPT_H

#include <stdlib.h>
#include <tsp_utilities.h>
#include <chrono.h>

void two_opt(tour* solution, instance* inst);
void shake_three_edges(tour* solution, instance* inst, int* elements_to_swap);
void shake_five_edges(tour* solution, instance* inst, int* elements_to_swap);
void reverse_segment(int* solution_path, int i, int j);

#endif // K_OPT