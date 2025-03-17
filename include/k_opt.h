#ifndef K_OPT_H
#define K_OPT_H

#include <stdlib.h>
#include <tsp_utilities.h>
#include <chrono.h>

void two_opt(int* solution, double cost, instance* inst);
void shake_three_edges(int* solution, instance* inst, int* elements_to_swap);
void shake_five_edges(int* solution, instance* inst, int* elements_to_swap);
void reverse_segment(int* solution, int i, int j);

#endif // K_OPT