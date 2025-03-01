#include <cplex.h>
#include <tsp_utilities.h>
#include <sys/time.h>

// Function declarations
void free_instance(instance *inst);
void parse_command_line(int argc, char **argv, instance *inst);
void read_input(instance *inst);

int main(int argc, char **argv) {
    if ( argc < 2 ) { printf("Wrong command line parameters\n"); exit(1); }       
	if ( VERBOSE >= 2 ) { for (int a = 0; a < argc; a++) printf("%s ", argv[a]); printf("\n"); }
    
    struct timeval start, end;
    double elapsed_time;

    gettimeofday(&start, NULL); // Start time

    instance inst;
    if ( VERBOSE >= 4000 ) { printf("Instance created!\n"); }
    
    // Parse the command line parameters
    parse_command_line(argc, argv, &inst);
    
    // Read the input file if it is defined otherwise generate random coordinates
    if ( VERBOSE >= 4000 ) { printf("Parse completed!\n"); }
    if(strcmp(inst.input_file, "NULL")) {
        if ( VERBOSE >= 100) {printf("Input file: %s\n", inst.input_file);}
        read_input(&inst);
    }
    else {
        if ( VERBOSE >= 100) { printf("Random generator\n"); }
        random_generator(&inst);
    }

    inst.solution = (int *) malloc(inst.nnodes * sizeof(int *));
    for (size_t i = 0; i < inst.nnodes; i++) {
        inst.solution[i] = i;
    }
    
    // Plot the solution
    plot_solution(&inst);
    
	if ( VERBOSE >= 1 ) {
        gettimeofday(&end, NULL); // End time
        elapsed_time = (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.0;
        printf("Execution time: %f seconds\n", elapsed_time);
	}

    free_instance(&inst);
    if ( VERBOSE >= 4000 ){ printf("Instance freed!\n"); }

    return 0;
}

/**
 * Free the instance memory
 * @param inst instance to be freed
 */
void free_instance(instance *inst) {
    free(inst->xcoord);
    free(inst->ycoord);
    free(inst->solution);
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
    inst->xcoord = NULL;
    inst->ycoord = NULL;
    inst->solution = NULL;
    
    int help = 0; if ( argc < 1 ) help = 1;// if no parameters, print help
    int node_flag = 1, number_nodes = 0;
	for ( int i = 1; i < argc; i++ ) { 
        if ( strcmp(argv[i],"-file") == 0 ) { strcpy(inst->input_file,argv[++i]); node_flag = 0; continue; }
		if ( strcmp(argv[i],"-input") == 0 ) { strcpy(inst->input_file,argv[++i]); continue; }
		if ( strcmp(argv[i],"-f") == 0 ) { strcpy(inst->input_file,argv[++i]); continue; } 	
		if ( strcmp(argv[i],"-time_limit") == 0 ) { inst->timelimit = atof(argv[++i]); continue; }
		if ( strcmp(argv[i],"-seed") == 0 ) { inst->seed = abs(atoi(argv[++i])); continue; }
        if ( strcmp(argv[i],"-nodes") == 0 ) { number_nodes = abs(atoi(argv[++i])); continue; }
        if ( strcmp(argv[i],"-help") == 0 ) { help = 1; continue; }
		if ( strcmp(argv[i],"--help") == 0 ) { help = 1; continue; } 
        help = 1;
    }

    if (node_flag) { inst->nnodes = number_nodes; }

    if ( help || (VERBOSE >= 10) ) {
		printf("\n\nAvailable parameters-------------------------------------------------------------------------\n");
		printf("-file %s\n", inst->input_file); 
		printf("-time_limit %lf\n", inst->timelimit); 
		printf("-seed %d\n", inst->seed); 
        printf("-nodes %d\n", inst->nnodes);
		printf("\nenter -help or --help for help\n");
		printf("----------------------------------------------------------------------------------------------\n\n");
	}        
	
	if ( help ) exit(1);
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
            inst->xcoord = (double *) malloc(inst->nnodes * sizeof(double));
            inst->ycoord = (double *) malloc(inst->nnodes * sizeof(double));
            inst->solution = (int *) malloc(inst->nnodes * sizeof(int));    
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
            inst->xcoord[i] = atof(token1);
            inst->ycoord[i] = atof(token2);
            continue;
        }

        if ( VERBOSE >= 1000 ) { (" Final active section: %d\n", active_section); }
        print_error("Wrong format for the current parser!\n");

    }

    fclose(file);
}