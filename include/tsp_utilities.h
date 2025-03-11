#ifndef TSP_UTILITIES_H
#define TSP_UTILITIES_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h> 
#include <chrono.h>
#include <cplex.h>

// Verbosity level
#define VERBOSE 1001

// Constants
#define MAX_BOUNDARY 10000
#define EPS_COST 1e-9
#define INF_COST 1e38

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
    char input_file[256];
    point* points;

    int* best_sol;
    double best_cost;
    double* cost;

    double timelimit;
    double tstart;

} instance;

void free_instance(instance *inst);
void read_input(instance *inst);
void parse_command_line(int argc, char** argv, instance *inst);
double random01();
void random_instance_generator(instance *inst);
void plot_solution(instance *inst, int* solution);
void print_error(const char *err); 
void compute_all_costs(instance* instance);
int check_sol(int* solution, double cost, instance* inst);
void update_best_sol(instance* inst, int* solution, double cost);
double dist(int i, int j, instance *inst);
double dist2(int i, int j, instance *inst);
void swap(int* arr, int i, int j);
void refinement_two_opt(int* solution, instance* inst);

#endif // TSP_UTILITIES_H