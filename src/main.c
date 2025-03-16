#include <tsp_utilities.h>
#include <sys/time.h>
#include <greedy.h>
#include <vns.h>

#include "grasp.h"


void print_menu() {
    printf("\nSelect the algorithm to run:\n");
    printf("1 - Nearest Neighbor\n");
    printf("2 - Nearest Neighbor + 2-OPT Refinement\n");
    printf("3 - Variable Neighborhood Search (VNS)\n");
    printf("4 - Tabu Search\n");
    printf("5 - Grasp Multi-Start\n");
    printf("Enter your choice: ");
}

int main(int argc, char **argv) {
    if ( argc < 2 ) { printf("Wrong command line parameters\n"); exit(1); }       
	if ( VERBOSE >= 2 ) { for (int a = 0; a < argc; a++) printf("%s ", argv[a]); printf("\n"); }
    
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

    int choice;
    print_menu();

    if (scanf("%d", &choice) != 1) {
        printf("Invalid input.\n");
        free_instance(&inst);
        exit(1);
    }

    double t1 = second(); // Start time
    switch (choice) {
        case 1:
            if (VERBOSE >= 1) { printf("Running Greedy Multi-Start...\n"); }
            if (greedy_multi_start(&inst, 0)) {
                print_error("Error in greedy_multi_start\n");
            }
            break;
        case 2:
            if (VERBOSE >= 1) { printf("Running Greedy Multi-Start + 2-OPT Refinement...\n"); }
            if (greedy_multi_start(&inst, 1)) {
                print_error("Error in greedy_multi_start\n");
            }
            break;
        case 3:
            if (VERBOSE >= 1) { printf("Running Variable Neighborhood Search (VNS)...\n"); }
            if (vns(&inst)) {
                print_error("Error in vns\n");
            }
            break;
        case 4:
            // Tabu Search
            break;
        case 5:
            if (VERBOSE >= 1) { printf("Running Grasp Multi-Start...\n"); }
            if (grasp_multi_start(&inst)) {
                print_error("Error in grasp_multi_start\n");
            }
            break;
        default:
            printf("Invalid choice.\n");
            printf("HEREEEE");
            break;
    }
    
    double t2 = second(); // End time
	if ( VERBOSE >= 1 ) {
        double elapsed_time = t2-t1;
        printf("Total execution time: %lf seconds\n", elapsed_time);
	}

    free_instance(&inst);
    if ( VERBOSE >= 4000 ){ printf("Instance freed!\n"); }

    return 0;
}
