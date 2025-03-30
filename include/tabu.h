#ifndef TABU_H
#define TABU_H

#include <tsp_utilities.h>
#include <greedy.h>
#include <stdbool.h>


int tabu(const instance* inst, tour* solution, double timelimit);


#endif //TABU_H
