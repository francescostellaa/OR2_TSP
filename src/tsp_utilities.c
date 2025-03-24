#include <tsp_utilities.h>

/**
 * Print error message and exit the program
 * @param err error message
 */
void print_error(const char *err) { printf("\n\n ERROR: %s \n\n", err); fflush(NULL); exit(1); } 

/**
 * Free the instance memory
 * @param inst instance to be freed
 */
void free_instance(instance *inst) {
    free(inst->points);
    free(inst->best_sol);
}

/**
 * Read the input file and store the coordinates of the nodes in the instance structure
 * @param inst instance to store the coordinates
 */
void read_input(instance *inst) {
    FILE *file = fopen(inst->input_file, "r");
    if (file == NULL) { print_error("Error opening file\n"); }

    inst->nnodes = -1;
    
    char line[256];
    char *par_name;
    char *token1;
    char *token2;

    int active_section = 0;

    while( fgets(line, sizeof(line), file) != NULL){

        if ( VERBOSE >= 2000 ) { printf("line: %s\n", line); fflush(NULL); }
        if ( strlen(line) < 2 ) continue; // empty line

        par_name = strtok(line, " :");

        if ( VERBOSE >= 3000 ) { printf("par_name: %s\n", par_name); fflush(NULL); }
        if ( strncmp(par_name, "NAME", 4) == 0 ) { active_section = 0; continue; }

        if ( strncmp(par_name, "COMMENT", 7) == 0 ) { 
            active_section = 0;
            token1 = strtok(NULL, "");
            if ( VERBOSE >= 10) { printf("COMMENT: %s\n", token1); fflush(NULL); }
            continue; 
        }

        if ( strncmp(par_name, "TYPE", 4) == 0 ) { 
            active_section = 0;
            token1 = strtok(NULL, " :");
            if ( VERBOSE >= 10) { printf("TYPE: %s\n", token1); fflush(NULL); }
            continue; 
        }

        if ( strncmp(par_name, "DIMENSION", 9) == 0 ) { 
            if ( inst->nnodes >= 0 ) { print_error("DIMENSION already defined\n"); }
            active_section = 0;
            token1 = strtok(NULL, " :");
            inst->nnodes = atoi(token1);
            if ( VERBOSE >= 1000 ) { printf("DIMENSION: %d\n", inst->nnodes); fflush(NULL); }
            inst->points = (point*)malloc(inst->nnodes * sizeof(point));
            inst->best_sol = (tour*)malloc(sizeof(tour));
            inst->best_sol->path = (int *) malloc((inst->nnodes + 1) * sizeof(int));
            inst->best_sol->cost = INF_COST;
            continue; 
        }

        if ( strncmp(par_name, "EDGE_WEIGHT_TYPE", 18) == 0 ) {
            active_section = 0;
            token1 = strtok(NULL, " :"); 
            if( strncmp(token1, "EUC_2D", 6) != 0 ) { print_error("EDGE_WEIGHT_TYPE different from euclidian!\n"); }
            continue;
        }

        if ( strncmp(par_name, "NODE_COORD_SECTION", 18) == 0 ) { 
            if ( inst->nnodes <= 0 ) { print_error("DIMENSION not defined\n"); }
            active_section = 1;
            continue; 
        }

        if ( strncmp(par_name, "EOF", 3) == 0 ) { 
            active_section = 0;
            break; 
        }
        
        if ( active_section == 1 ) {
            int i = atoi(par_name) - 1;
            token1 = strtok(NULL, " :");
            token2 = strtok(NULL, " :");
            if ( VERBOSE >= 10000) { printf("Coordinates: x: %s, y: %s\n", token1, token2); fflush(NULL); }
            inst->points[i].x = atof(token1);
            inst->points[i].y = atof(token2);
            continue;
        }

        if ( VERBOSE >= 1000 ) { printf("Final active section: %d\n", active_section); }
        print_error("Wrong format for the current parser!\n");

    }

    fclose(file);
}

/**
 * Parse the command line parameters and set the instance values
 * @param argc number of parameters
 * @param argv array of parameters
 * @param inst instance to be set
 */
void parse_command_line(int argc, char** argv, instance *inst) { 
	
	if ( VERBOSE >= 100 ) printf("Running %s with %d parameters!\n", argv[0], argc-1);
    
	// default   
	strcpy(inst->input_file, "NULL");
	inst->seed = 0; 
	inst->timelimit = CPX_INFBOUND;     
    inst->nnodes = 0;
    inst->points = NULL;
    inst->best_sol = (tour*)malloc(sizeof(tour));
    inst->best_sol->path = NULL;
    inst->best_sol->cost = INF_COST;

    int help = 0; if ( argc < 1 ) help = 1;// if no parameters, print help
    int node_flag = 1, number_nodes = 0;
	for ( int i = 1; i < argc; i++ ) { 
        if ( strcmp(argv[i],"-file") == 0 ) { strcpy(inst->input_file,argv[++i]); node_flag = 0; continue; }
		if ( strcmp(argv[i],"-input") == 0 ) { strcpy(inst->input_file,argv[++i]); node_flag = 0; continue; }
		if ( strcmp(argv[i],"-f") == 0 ) { strcpy(inst->input_file,argv[++i]); node_flag = 0; continue; } 	
		if ( strcmp(argv[i],"-time_limit") == 0 ) { inst->timelimit = atof(argv[++i]); continue; }
		if ( strcmp(argv[i],"-seed") == 0 ) { inst->seed = abs(atoi(argv[++i])); continue; }
        if ( strcmp(argv[i],"-nodes") == 0 ) { 
            if (++i >= argc) {
                print_error("Error: Missing value for -nodes\n");
            }
            number_nodes = abs(atoi(argv[i])); 
            continue;
        }
        if ( strcmp(argv[i],"-help") == 0 ) { help = 1; continue; }
		if ( strcmp(argv[i],"--help") == 0 ) { help = 1; continue; } 
        help = 1;
    }

    if (node_flag) { 
        if (number_nodes <= 0) { print_error("Number of nodes not defined\n"); }
        else{
            inst->nnodes = number_nodes; 
        }
    }

    if ( help || (VERBOSE >= 10) ) {
		printf("\n\nAvailable parameters-------------------------------------------------------------------------\n");
		printf("-file %s\n", inst->input_file); 
		printf("-time_limit %lf\n", inst->timelimit); 
		printf("-seed %d\n", inst->seed); 
        if (node_flag){
            printf("-nodes %d\n", inst->nnodes);
        }
		printf("\n enter -help or --help for help\n");
		printf("----------------------------------------------------------------------------------------------\n\n");
	}        
	
	if ( help ) exit(1);
}

/**
 * Generate a random number between 0 and 1
 * @return random number
 */
double random01() { return ((double) rand() / RAND_MAX); }

/**
 * Generate random coordinates for the nodes of the TSP instance
 * @param inst instance to store the coordinates
 */
void random_instance_generator(instance *inst) {
    srand(inst->seed);
    inst->points = (point*) malloc(inst->nnodes * sizeof(point));
    inst->best_sol = (tour*)malloc(sizeof(tour));
    inst->best_sol->path = (int *) malloc((inst->nnodes + 1) * sizeof(int));
    inst->best_sol->cost = INF_COST;


    if ( VERBOSE >= 1000) { printf("Number of Nodes: %d\n", inst->nnodes); fflush(NULL); };
    for (int i = 0; i < inst->nnodes; i++){
        inst->points[i].x = random01() * MAX_BOUNDARY;
        inst->points[i].y = random01() * MAX_BOUNDARY;
    }
}

/**
 * Compute the Euclidean distance between two nodes
 * @param i index of the first node
 * @param j index of the second node
 * @param inst instance with the nodes
 * @return distance between the nodes
 */
double dist(int i, int j, instance *inst) {
    return (sqrt(pow(inst->points[i].x - inst->points[j].x, 2) + pow(inst->points[i].y - inst->points[j].y, 2)));
}

/**
 * Compute the squared Euclidean distance between two nodes
 * @param i index of the first node
 * @param j index of the second node
 * @param inst instance with the nodes
 * @return squared distance between the nodes
 */
double dist2(int i, int j, instance *inst) {
    return (pow(inst->points[i].x - inst->points[j].x, 2) + pow(inst->points[i].y - inst->points[j].y, 2));
}

/**
 * Compute the cost of all edges in the instance
 * @param instance instance with the nodes
 */
void compute_all_costs(instance* instance) {
    int n = instance->nnodes;
    instance->cost_matrix = (double*)malloc(n * n * sizeof(double));

    for (int i = 0; i < n; i++) {
        for (int j = i; j < n; j++) { 
            double d = dist(i, j, instance);
            if (strcmp(instance->input_file, "NULL") != 0) {
                instance->cost_matrix[i * n + j] = round(d);
                instance->cost_matrix[j * n + i] = round(d);
            } else {
                instance->cost_matrix[i * n + j] = d;
                instance->cost_matrix[j * n + i] = d;
            }
        }
    }
}

/**
 * Check if the solution is feasible
 * @param solution_path  solution path
 * @param cost cost of the solution
 * @return 1 if the solution is feasible, 0 otherwise
 */
int check_sol(int* solution_path, double cost, instance* inst){
    
    int* count = calloc(inst->nnodes, sizeof(int));
    int n = inst->nnodes;

    if (solution_path[n] != solution_path[0]){
        print_error("First and last nodes are different\n");
    }

    for (int i = 0; i < inst->nnodes; i++){
        count[solution_path[i]]++;
    }

    for (int i = 0; i < n; i++){
        if (count[i] != 1){
            printf("Node %d appears %d times in the solution\n", i, count[i]);
            free(count);
            print_error("Exiting...\n");
        }
    }

    free(count);

    double total_cost = 0;
    for (int i = 0; i < n; i++){
        total_cost += inst->cost_matrix[solution_path[i] * n + solution_path[i+1]];
    }

    if (fabs(total_cost - cost) > EPS_COST){
        print_error("Computed cost is different from the input cost\n");
    }

    return 1;
}

/**
 * Update the best solution found so far
 * @param inst instance with the solution
 * @param solution array with the solution
 */
void update_best_sol(instance* inst, tour* solution){
    if (inst->best_sol->cost > solution->cost) {
        check_sol(solution->path, solution->cost, inst);
        memcpy(inst->best_sol->path, solution->path, ((inst->nnodes)+1) * sizeof(int));
        inst->best_sol->cost = solution->cost;
    }
}

/**
 * Compute the cost of a solution
 * @param solution array with the solution
 * @param inst instance with the solution
 * @return cost of the solution
 */
void compute_solution_cost(tour* solution, instance* inst) {
    solution->cost = 0.0;
    for (int i = 0; i < inst->nnodes; i++) {
        solution->cost += inst->cost_matrix[solution->path[i] * inst->nnodes + solution->path[i + 1]];
    }
}

/**
 * Swap two elements in an array
 * @param arr array with the elements
 * @param i index of the first element
 * @param j index of the second element
 */
void swap(int* arr, int i, int j) {
    if (i != j) {
        int temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
    }
}

/**
 * Reverse the segment between start and end in the solution
 */
void reverse_segment(int* solution_path, int start, int end) {
    while (start < end) {
        swap(solution_path, start, end);
        start++;
        end--;
    }
}


/**
 * Plot the solution using gnuplot and save the output as a PNG file
 * @param inst instance with the solution to be plotted
 */
void plot_solution(instance *inst, int* solution) {
    FILE *gnuplot = popen("gnuplot -persist", "w");
    if (gnuplot == NULL) { 
        printf("Error opening gnuplot\n");
        return;
    }

    // Set up the Gnuplot configuration
    fprintf(gnuplot, "set terminal pngcairo enhanced color font 'Helvetica,12' size 1000,720\n");  // Set PNG output
    fprintf(gnuplot, "set output '../data/solution.png'\n");
    fprintf(gnuplot, "set title 'TSP Solution' font 'Helvetica,16\n");

    // Define the style
    fprintf(gnuplot, "set style line 1 lc rgb '#FF0000' lt 1 lw 2 pt 7 ps 1.5\n");

    // Plot command
    fprintf(gnuplot, "plot '-' with linespoints ls 1 title 'TSP solution'\n");

    // Loop through the solution sequence
    for (int i = 0; i < (inst->nnodes + 1); i++) {
        int node = solution[i];  // Get the node index
        fprintf(gnuplot, "%lf %lf\n", inst->points[node].x, inst->points[node].y);
    }

    // Close the tour by adding the first node at the end
    fprintf(gnuplot, "%lf %lf\n", inst->points[solution[0]].x, inst->points[solution[0]].y);

    // End data input
    fprintf(gnuplot, "e\n");

    fflush(gnuplot);
    pclose(gnuplot);  // Close Gnuplot properly
}

/**
 * Save the history of the incumbent solution
 * @param best_cost cost of the best solution
 */
void save_history_incumbent(double best_cost) {
    static int first_call = 0;
    static int iteration_counter = 0;

    FILE *file = NULL;

    // First call: erase file contents
    if (!first_call) {
        file = fopen("../data/incumbent.txt", "w");
        first_call = 1;
    } else {
        file = fopen("../data/incumbent.txt", "a");
    }

    if (file == NULL) {
        perror("Error opening file");
        return;
    }

    // Write current iteration and best cost to file
    fprintf(file, "%d %.6f\n", iteration_counter++, best_cost);
    fclose(file);
}


/**
 * Save the history of the cost
 * @param cost cost of the solution
 */
void save_history_cost(double cost) {
    static int first_call = 0;
    static int iteration_counter = 0;

    FILE *file;

    // First time opening the file, erase its content
    if (!first_call) {
        file = fopen("../data/cost.txt", "w");
        first_call = 1;
    } else {
        file = fopen("../data/cost.txt", "a");
    }

    if (file == NULL) {
        perror("Error opening file");
        return;
    }

    // Compute new index and write to file
    fprintf(file, "%d %.6f\n", iteration_counter++, cost);
    fclose(file);
}

/**
 * Plot the history of the cost
 */
void plot_history_cost() {
    FILE *gnuplot = popen("gnuplot -persist", "w");
    if (gnuplot == NULL) {
        printf("Error opening gnuplot\n");
        return;
    }

    // Set up Gnuplot settings
    fprintf(gnuplot, "set terminal pngcairo enhanced color font 'Helvetica,12' size 1080,720\n");
    fprintf(gnuplot, "set output '../data/cost_evolution.png'\n");
    fprintf(gnuplot, "set title 'Evolution of Best Solution' font 'Helvetica,16'\n");

    fprintf(gnuplot, "set xlabel 'Iteration'\n");
    fprintf(gnuplot, "set ylabel 'Cost'\n");
    fprintf(gnuplot, "set grid\n");

    // Define the style for the plot
    fprintf(gnuplot, "set style line 1 lc rgb '#FF0000' lt 1 lw 2 pt 7 ps 1.5\n");

    // Plot command, reading from the file
    fprintf(gnuplot, "plot '../data/cost.txt' using 1:2 with lines ls 1 title 'Cost evolution'\n");

    fflush(gnuplot);
    pclose(gnuplot);
}

/**
 * Plot the evolution of the incumbent solution for the TSP
 */
void plot_incumbent() {
    FILE *gnuplot = popen("gnuplot -persist", "w");
    if (gnuplot == NULL) { 
        printf("Error opening gnuplot\n");
        return;
    }

    // Set up Gnuplot settings
    fprintf(gnuplot, "set terminal pngcairo enhanced color font 'Helvetica,12' size 1080,720\n");  
    fprintf(gnuplot, "set output '../data/solution_evolution.png'\n");
    fprintf(gnuplot, "set title 'Evolution of Best Solution' font 'Helvetica,16'\n");
    
    fprintf(gnuplot, "set xlabel 'Iteration'\n");
    fprintf(gnuplot, "set ylabel 'Best Cost'\n");
    fprintf(gnuplot, "set grid\n");

    // Define the style for the plot
    fprintf(gnuplot, "set style line 1 lc rgb '#FF0000' lt 1 lw 2 pt 7 ps 1.5\n");

    // Plot command, reading from the file
    fprintf(gnuplot, "plot '../data/incumbent.txt' using 1:2 with lines ls 1 title 'Best Cost'\n");

    fflush(gnuplot);
    pclose(gnuplot);  // Close Gnuplot properly
}

void plot_incumbent_and_costs() {
    FILE *gnuplot = popen("gnuplot -persist", "w");
    if (gnuplot == NULL) { 
        printf("Error opening gnuplot\n");
        return;
    }

    // Set up Gnuplot settings
    //fprintf(gnuplot, "set terminal pngcairo enhanced color font 'Helvetica,12'\n");  
    fprintf(gnuplot, "set terminal pngcairo enhanced color font 'Helvetica,12' size 1080,720\n"); 
    fprintf(gnuplot, "set output '../data/complete_solution_evolution.png'\n");
    fprintf(gnuplot, "set title 'Evolution of Best Solution' font 'Helvetica,16'\n");
    
    fprintf(gnuplot, "set xlabel 'Iteration'\n");
    fprintf(gnuplot, "set ylabel 'Cost'\n");
    fprintf(gnuplot, "set grid\n");

    // Define styles
    fprintf(gnuplot, "set style line 1 lc rgb '#0000FF' lt 1 lw 1 pt 6 ps 1\n");  
    fprintf(gnuplot, "set style line 2 lc rgb '#FF0000' lt 1 lw 3 pt 7 ps 1.5\n"); 

    // Plot both files
    fprintf(gnuplot, "plot '../data/cost.txt' using 1:2 with lines ls 1 title 'Cost', \\\n");
    fprintf(gnuplot, "     '../data/incumbent.txt' using 1:2 with lines ls 2 title 'Best Cost'\n");

    fflush(gnuplot);
    pclose(gnuplot);
}


