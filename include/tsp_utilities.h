#ifndef TSP_UTILITIES_H
#define TSP_UTILITIES_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h> 
#include <chrono.h>
#include <cplex.h>
#include <assert.h>

// Verbosity level
#define VERBOSE 1001

// Constants
#define MAX_BOUNDARY 10000
#define EPS_COST 1e-8
#define INF_COST 1e38

/**
 * Point structure
 */
typedef struct {
    double x;
    double y;
} point;

/**
 * Generic solution structure
 */
typedef struct {
    int* path;
    double cost;
} tour;


/**
 * TSP instance structure 
 */
typedef struct {
    int nnodes;
    int seed;
    char input_file[256]; // input file name
    point* points; // points coordinates
    int integer_costs; // 1 if costs are integer, 0 otherwise
    int ncols;

    double* cost_matrix; // cost matrix
    tour* best_sol;

    double timelimit;
    double tstart;
    double prob_hard_fixing;

    int mode;

} instance;

/*
 * Parameters for the algorithms
 */
typedef struct {

    int num_kicks;
    int interval_tenure;
    float tenure_scaling;
    double prob_grasp;
    int k_neighborhood;
    int population_size;
    
} parameters;

void free_instance(instance *inst);
void read_input(instance *inst);
void parse_command_line(int argc, char** argv, instance *inst, parameters *params, int *alg);
double random01();
void random_instance_generator(instance *inst);
void plot_solution(const instance *inst, int* solution);
void print_error(const char *err); 
void compute_all_costs(instance* instance);
int check_sol(int* solution, double cost, const instance* inst);
void update_best_sol(instance* inst, tour* solution);
double dist(int i, int j, instance *inst);
double dist2(int i, int j, instance *inst);
void swap(int* arr, int i, int j);
void reverse_segment(int* solution_path, int i, int j);
void compute_solution_cost(tour* solution, const instance* inst);
void save_history_incumbent(double best_cost, const char* filename);
void plot_incumbent(const char *input_filename, const char *output_filename);
void save_history_cost(double cost, const char* filename);
void plot_history_cost(const char *input_filename, const char *output_filename);
void plot_incumbent_and_costs(const char *cost_filename, const char *incumbent_filename, const char *output_filename);

#endif // TSP_UTILITIES_H