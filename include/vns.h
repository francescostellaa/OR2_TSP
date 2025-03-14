#ifndef VNS_H
#define VNS_H

#include <stdlib.h>
#include <tsp_utilities.h>
#include <chrono.h>
#include <greedy.h>

#define MAX_ITERATIONS_VNS 250

int vns(instance* inst);

#endif // VNS_H