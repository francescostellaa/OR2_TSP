#ifndef VNS_H
#define VNS_H

#include <stdlib.h>
#include <tsp_utilities.h>
#include <chrono.h>
#include <greedy.h>
#include <k_opt.h>

int vns(const instance* inst, tour* solution, double timelimit, int num_kicks);
int vns_multiparam(instance* inst, tour* solution, double timelimit, int* num_kicks_params, int num_kicks_size, char* output_file);

#endif // VNS_H