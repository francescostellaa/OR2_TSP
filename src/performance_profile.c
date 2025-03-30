#include <performance_profile.h>
#include <greedy.h>
#include <vns.h>

int performance_profile(){

    // Create output directory if it doesn't exist
    char dirname[100];
    sprintf(dirname, "../data/perfprof/");

    // Create directory
    char mkdir_cmd[150];
    sprintf(mkdir_cmd, "mkdir -p %s", dirname);
    system(mkdir_cmd);

    // ========== VNS ==========

    int num_kicks_params[] = {0, 3, 5, 7, 10, 15};
    int num_params = sizeof(num_kicks_params) / sizeof(num_kicks_params[0]);

    FILE *file = fopen("../data/perfprof/perfprof_vns.csv", "w");
    if (file == NULL) {
        printf("Error opening file for writing\n");
        return 1;
    }

    // Write the header of the CSV file
    fprintf(file, "%d", num_params);
    for (int i = 0; i < num_params; i++) {
        fprintf(file, ";algo_%d", i);
    }
    fprintf(file, "\n");
    fclose(file);

    for (int i = 0; i < 10; i++){
        instance inst;
        inst.seed = i;
        inst.nnodes = 1000;
        inst.timelimit = 10;
        inst.tstart = second();
        strcpy(inst.input_file, "NULL");
        inst.best_sol = (tour*)malloc(sizeof(tour));
        inst.best_sol->path = NULL;
        inst.best_sol->cost = INF_COST;
        random_instance_generator(&inst);
        compute_all_costs(&inst);

        tour* solution_vns = malloc(sizeof(tour));
        solution_vns->path = (int*)malloc((inst.nnodes + 1) * sizeof(int));
        solution_vns->cost = 0.0;
        greedy(rand() % inst.nnodes, solution_vns, 0, &inst);

        if (vns_multiparam(&inst, solution_vns, inst.timelimit, num_kicks_params, num_params, "../data/perfprof/perfprof_vns.csv")) {
            print_error("Error in vns\n");
        }
        
        free(solution_vns->path);
        free(solution_vns);
        free_instance(&inst);

    }

    return 0;

}