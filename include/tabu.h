#ifndef TABU_H
#define TABU_H

#include <tsp_utilities.h>
#include <greedy.h>
#include <stdbool.h>

//#define TENURE 18

typedef struct {
    int i, j;
} tabu_move;

int tabu(instance* inst);


#endif //TABU_H
