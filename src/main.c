#include <tsp_utilities.h>
#include <greedy.h>
#include <vns.h>
#include <tabu.h>
#include <grasp.h>
#include <cplex_model.h>
#include <matheuristics.h>
#include <genetic.h>
#include <extra_mileage.h>

int main(int argc, char **argv) {
    if ( argc < 2 ) { printf("Wrong command line parameters\n"); exit(1); }       
	if ( VERBOSE >= 2 ) { for (int a = 0; a < argc; a++) printf("%s ", argv[a]); printf("\n"); }
    
    instance inst;
    parameters params;
    int alg = -1;
    if ( VERBOSE >= 4000 ) { printf("Instance allocated!\n"); }
    
    // Parse the command line parameters
    parse_command_line(argc, argv, &inst, &params, &alg);
    if ( VERBOSE >= 4000 ) { printf("Parse completed!\n"); }

    // Read the input file if it is defined otherwise generate random coordinates
    if(strcmp(inst.input_file, "NULL") != 0) {
        if ( VERBOSE >= 100) {printf("Input file: %s\n", inst.input_file);}
        read_input(&inst);
    }
    else {
        if ( VERBOSE >= 100) { printf("Random generator\n"); }
        random_instance_generator(&inst);
    }
    
    compute_all_costs(&inst);
    if ( VERBOSE >= 4000 ) { printf("Costs computed!\n"); }

    double t1 = second(); // Start time
    inst.tstart = t1;
    srand(inst.seed);   //Set the random seed
    
    switch (alg) {
        case 1:
            if (VERBOSE >= 1) { printf("Running Greedy Multi-Start...\n"); }
            if (greedy_multi_start(&inst, 0, inst.timelimit)) {
                print_error("Error in greedy_multi_start\n");
            }
            break;
        case 2:
            if (VERBOSE >= 1) { printf("Running Greedy Multi-Start + 2-OPT Refinement...\n"); }
            if (greedy_multi_start(&inst, 1, inst.timelimit)) {
                print_error("Error in greedy_multi_start\n");
            }
            break;
        case 3:
            if (VERBOSE >= 1) { printf("Running Variable Neighborhood Search (VNS)...\n"); }
            tour* solution_vns = malloc(sizeof(tour));
            solution_vns->path = (int*)malloc((inst.nnodes + 1) * sizeof(int));
            solution_vns->cost = 0.0;
            greedy(rand() % inst.nnodes, solution_vns, 0, &inst);
            if (vns(&inst, solution_vns, inst.timelimit, params.num_kicks)) {
                print_error("Error in vns\n");
            }
            update_best_sol(&inst, solution_vns);
            if(VERBOSE >= 1) { printf("Best cost: %lf\n", inst.best_sol->cost); }
            free(solution_vns->path);
            free(solution_vns);
            break;
        case 4:
            if (VERBOSE >= 1) { printf("Running Tabu Search...\n"); }
            tour* solution_tabu = malloc(sizeof(tour));
            solution_tabu->path = (int*)malloc((inst.nnodes + 1) * sizeof(int));
            solution_tabu->cost = 0.0;
            greedy(rand() % inst.nnodes, solution_tabu, 0, &inst);

            if (tabu(&inst, solution_tabu, inst.timelimit, params.interval_tenure, params.tenure_scaling)) {
                print_error("Error in tabu\n");
            }
            update_best_sol(&inst, solution_tabu);
            if(VERBOSE >= 1) { printf("Best cost: %lf\n", inst.best_sol->cost); }
            free(solution_tabu->path);
            free(solution_tabu);
            break;
        case 5:
            if (VERBOSE >= 1) { printf("Running Grasp Multi-Start...\n"); }
            if (grasp_multi_start(&inst, inst.timelimit, params.prob_grasp)) {
                print_error("Error in grasp_multi_start\n");
            }
            break;
        case 6:
            if (VERBOSE >= 1) { printf("Running Branch and Cut...\n"); }
            if (TSPopt(&inst, 6)) {
                print_error("Error in TSPopt\n");
            }
            break;
        case 7:
            if (VERBOSE >= 1) { printf("Running Benders...\n"); }
            if (TSPopt(&inst, 7)) {
                print_error("Error in TSPopt\n");
            }
            break;
        case 8:
            if (VERBOSE >= 1) { printf("Running Hard-Fixing...\n"); }
            if (hard_fixing(&inst, &params)) {
                print_error("Error in Hard-Fixing\n");
            }
            break;
        case 9:
            if (VERBOSE >= 1) { printf("Running Local Branching...\n"); }
            if (local_branching(&inst, &params)) {
                print_error("Error in Local Branching\n");
            }
            break;
        case 10:
            if (VERBOSE >= 1) { printf("Running Genetic Algorithm...\n"); }
            tour* solution_genetic = malloc(sizeof(tour));
            solution_genetic->path = (int*)malloc((inst.nnodes + 1) * sizeof(int));
            solution_genetic->cost = INF_COST;
            if (genetic_algorithm(&inst, solution_genetic, params.population_size, inst.timelimit)) {
                print_error("Error in vns\n");
            }
            update_best_sol(&inst, solution_genetic);
            if(VERBOSE >= 1) { printf("Best cost: %lf\n", inst.best_sol->cost); }
            free(solution_genetic->path);
            free(solution_genetic);
            break;
        case 11:
            if (VERBOSE >= 1) { printf("Running Extra Mileage...\n"); }
            if (extra_mileage_multi_start(&inst, 1, inst.timelimit)) {
                print_error("Error in extra mileage\n");
            }
            break;
        default:
            printf("Invalid choice.\n");
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
