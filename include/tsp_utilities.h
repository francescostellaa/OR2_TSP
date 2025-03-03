#ifndef TSP_UTILITIES_H
#define TSP_UTILITIES_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h> 

// Verbosity level
#define VERBOSE 100
#define MAX_BOUNDARY 10000

/**
 * Point structure
 */
typedef struct {
    double x;
    double y;
} point;

/**
 * TSP instance structure 
 */
typedef struct {
    int nnodes;
    int seed;
    double timelimit;
    char input_file[256];
    point* points;
    int* best_solution;
} instance;

double random01();
void random_instance_generator(instance *inst);
void plot_solution(instance *inst, int* solution);
void print_error(const char *err); 

#endif // TSP_UTILITIES_H