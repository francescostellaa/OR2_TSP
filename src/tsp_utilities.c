#include <tsp_utilities.h>

/**
 * Print error message and exit the program
 * @param err error message
 */
void print_error(const char *err) { printf("\n\n ERROR: %s \n\n", err); fflush(NULL); exit(1); } 

/**
 * Generate random coordinates for the nodes of the TSP instance
 * @param inst instance to store the coordinates
 */
void random_generator(instance *inst) {
    srand(inst->seed);
    int NODE_NUMBER = inst->nnodes;
    inst->xcoord = (double *) malloc(NODE_NUMBER * sizeof(double));
    inst->ycoord = (double *) malloc(NODE_NUMBER * sizeof(double));
    inst->solution = (int *) malloc(NODE_NUMBER * sizeof(int *));

    if ( VERBOSE >= 1000) { printf("Number of Nodes: %d\n", NODE_NUMBER); fflush(NULL); };
    for (size_t i = 0; i < NODE_NUMBER; i++){
        inst->xcoord[i] = round(rand() % NODE_NUMBER);
        inst->ycoord[i] = round(rand() % NODE_NUMBER);
    }

    for (int i = 0; i < 10; i++) {
        inst->solution[i] = i;
    }
    
    return;
}

/**
 * Plot the solution using gnuplot and save the output as a PNG file
 * @param inst instance with the solution to be plotted
 */
void plot_solution(instance *inst) {
    FILE *gnuplot = popen("gnuplot -persist", "w");
    if (gnuplot == NULL) { 
        printf("Error opening gnuplot\n");
        return;
    }

    // Set up the Gnuplot configuration
    fprintf(gnuplot, "set terminal pngcairo\n");  // Set PNG output
    fprintf(gnuplot, "set output '../data/solution.png'\n");
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
