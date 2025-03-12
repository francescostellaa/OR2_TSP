#ifndef K_OPT_H
#define K_OPT_H

#include <stdlib.h>
#include <tsp_utilities.h>
#include <chrono.h>

void two_opt(int* solution, instance* inst);
void three_opt(int* solution, instance* inst, int* elements_to_swap);

#endif // K_OPT