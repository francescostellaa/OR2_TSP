#include <tsp_utilities.h>
#include <stdlib.h>

/**
 * Print error message and exit
 */
void print_error(const char *err) { printf("\n\n ERROR: %s \n\n", err); fflush(NULL); exit(1); } 

/**
 * Generate random coordinates for the nodes
 */
void random_generator(instance *inst) {
    srand(inst->seed);
    int XY_SIZE = inst->nnodes;
    inst->xcoord = (double *) malloc(inst->nnodes * sizeof(double));
    inst->ycoord = (double *) malloc(inst->nnodes * sizeof(double));
    inst->solution = (int *) malloc(inst->nnodes * sizeof(int *));
    if ( VERBOSE >= 1000) { printf("XY_SIZE: %d\n", XY_SIZE); fflush(NULL); };
    for (size_t i = 0; i < XY_SIZE; i++){
        inst->xcoord[i] = round(rand() % XY_SIZE);
        inst->ycoord[i] = round(rand() % XY_SIZE);
    }

    for (int i = 0; i < 10; i++) {
        inst->solution[i] = i;
    }
    
    return;
}

/**
 * Plot the solution using gnuplot
 */
void plot_solution(instance *inst) {
    FILE *gnuplot = popen("gnuplot -persist", "w");
    if (gnuplot == NULL) { 
        printf("Error opening gnuplot\n");
        return;
    }

    // Set up the Gnuplot configuration
    fprintf(gnuplot, "set terminal pngcairo\n");  // Set PNG output
    fprintf(gnuplot, "set output 'solution.png'\n");
    fprintf(gnuplot, "set title 'TSP Solution'\n");

    // Define the style
    fprintf(gnuplot, "set style line 1 lc rgb '#FF0000' lt 1 lw 2 pt 7 ps 2\n");

    // Plot command
    fprintf(gnuplot, "plot '-' with linespoints ls 1 title 'TSP Solution'\n");

    // Loop through the solution sequence
    for (int i = 0; i < inst->nnodes; i++) {
        int node = inst->solution[i];  // Get the node index
        fprintf(gnuplot, "%lf %lf\n", inst->xcoord[node], inst->ycoord[node]);
    }

    // Close the tour by adding the first node at the end
    fprintf(gnuplot, "%lf %lf\n", inst->xcoord[inst->solution[0]], inst->ycoord[inst->solution[0]]);

    // End data input
    fprintf(gnuplot, "e\n");

    fflush(gnuplot);
    pclose(gnuplot);  // Close Gnuplot properly

    return;
}
