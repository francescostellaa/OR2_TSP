#ifndef GENETIC_H
#define GENETIC_H

#include <tsp_utilities.h>
#include <k_opt.h>
#include <stdbool.h>

void initialize_population(tour* population, int pop_size, instance* inst);
tour* tournament_selection(tour* population, int pop_size);
void crossover(const int* parent1, const int* parent2, int* child, int n);
void repair_solution(int* path, const instance* inst);
int genetic_algorithm(instance* inst, tour* best_solution, int pop_size, double timelimit);
void mutate(int* path, int n);
int compare_tours(const void* a, const void* b);

#endif //GENETIC_H