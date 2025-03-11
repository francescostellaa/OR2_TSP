#include <tsp_utilities.h>
#include <sys/time.h>
#include <greedy.h>

// Function declarations
double second();

int main(int argc, char **argv) {
    if ( argc < 2 ) { printf("Wrong command line parameters\n"); exit(1); }       
	if ( VERBOSE >= 2 ) { for (int a = 0; a < argc; a++) printf("%s ", argv[a]); printf("\n"); }
    
    double t1 = second(); // Start time
    instance inst;
    if ( VERBOSE >= 4000 ) { printf("Instance allocated!\n"); }
    
    // Parse the command line parameters
    parse_command_line(argc, argv, &inst);
    if ( VERBOSE >= 4000 ) { printf("Parse completed!\n"); }

    // Read the input file if it is defined otherwise generate random coordinates
    if(strcmp(inst.input_file, "NULL")) {
        if ( VERBOSE >= 100) {printf("Input file: %s\n", inst.input_file);}
        read_input(&inst);
    }
    else {
        if ( VERBOSE >= 100) { printf("Random generator\n"); }
        random_instance_generator(&inst);
    }
    
    compute_all_costs(&inst);
    if ( VERBOSE >= 4000 ) { printf("Costs computed!\n"); }
    
    if (greedy_multi_start(&inst)) {
        print_error("Error in greedy_multi_start\n");
    }
    
    double t2 = second(); // End time
	if ( VERBOSE >= 1 ) {
        double elapsed_time = t2-t1;
        printf("Execution time: %lf seconds\n", elapsed_time);
	}

    free_instance(&inst);
    if ( VERBOSE >= 4000 ){ printf("Instance freed!\n"); }

    return 0;
}
