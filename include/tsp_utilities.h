#ifndef TSP_UTILITIES_H
#define TSP_UTILITIES_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h> 

// Verbosity level
#define VERBOSE	        100

/**
 * TSP instance structure 
 */
typedef struct {
    int nnodes;
    int seed;
    double timelimit;
    char input_file[256];
    double *xcoord;
    double *ycoord;
    int* solution;
} instance;

void random_generator(instance *inst);
void plot_solution(instance *inst);
void print_error(const char *err); 

#endif // TSP_UTILITIES_H