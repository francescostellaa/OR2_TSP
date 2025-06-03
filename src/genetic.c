#include <genetic.h>

/**
 * Initialize the population for the genetic algorithm
 * @param population array of tours representing the population
 * @param pop_size size of the population
 * @param inst instance containing the problem data
 */
void initialize_population(tour* population, int pop_size, instance* inst){

    int n = inst->nnodes;

    for (int i = 0; i < pop_size; i++) {

        population[i].path = malloc((n+1) * sizeof(int));
        for (int j = 0; j < n; j++) {
            population[i].path[j] = j; 
        }

        for (int j = n - 1; j > 0; --j) {
            int k = rand() % (j + 1);
            int temp = population[i].path[j];
            population[i].path[j] = population[i].path[k];
            population[i].path[k] = temp;
        }

        population[i].path[n] = population[i].path[0];

        compute_solution_cost(&population[i], inst);

    }

}

/**
 * Select a tour from the population using tournament selection
 * @param population array of tours representing the population
 * @param pop_size size of the population
 * @return pointer to the selected tour
 */
tour* tournament_selection(tour* population, int pop_size) {
    int a = rand() % pop_size;
    int b = rand() % pop_size;
    return (population[a].cost < population[b].cost) ? &population[a] : &population[b];
}

/**
 * Perform crossover between two parent tours to create a child tour
 * @param parent1 first parent tour
 * @param parent2 second parent tour
 * @param child resulting child tour
 * @param n number of nodes in the tours
 */
void crossover(const int* parent1, const int* parent2, int* child, int n) {
    int start = rand() % n;
    int end = rand() % n;

    if (start > end) {
        int temp = start;
        start = end;
        end = temp;
    }

    int* taken = calloc(n, sizeof(int));

    for (int i = start; i <= end; i++) {
        child[i] = parent1[i];
        taken[child[i]] = 1;
    }

    int idx_child = (end + 1) % n;
    int idx_parent2 = (end + 1) % n;

    while (idx_child != start) {
        if (!taken[parent2[idx_parent2]]) {
            child[idx_child] = parent2[idx_parent2];
            taken[parent2[idx_parent2]] = 1;
            idx_child = (idx_child + 1) % n;
        }
        idx_parent2 = (idx_parent2 + 1) % n;
    }

    free(taken);

}

/**
 * Repair the solution by removing repeated nodes and adding missing nodes
 * @param path array representing the tour path
 * @param inst instance containing the problem data
 */
void repair_solution(int* path, const instance* inst) {
    
    int n = inst->nnodes;
    int* visited = calloc(n, sizeof(int));

    int* temp = malloc((n+1) * sizeof(int));
    int idx = 0;

    // Removing repeated nodes
    for (int i = 0; i < n; i++) {
        if (!visited[path[i]]) {
            visited[path[i]] = 1;
            temp[idx++] = path[i];
        }
    }

    // Extra mileage to add missing nodes
    for (int v = 0; v < n; v++) {

        if (!visited[v]) {
            
            double best_delta = INF_COST;
            int best_idx = -1;

            for (int i = 0; i < idx; i++) {

                double delta = (inst->cost_matrix[temp[i] * n + v] + inst->cost_matrix[v * n + temp[(i + 1) % idx]])
                                - (inst->cost_matrix[temp[i] * n + temp[(i + 1) % idx]]);
                
                if (delta < best_delta - EPS_COST) {
                    best_delta = delta;
                    best_idx = i + 1;
                }

            }

            for (int i = idx; i > best_idx; i--) {
                temp[i] = temp[i - 1];
            }

            temp[best_idx] = v;
            idx++;
            visited[v] = 1;

        }

    }

    for (int i = 0; i < n; i++) {
        path[i] = temp[i];
    }
    path[n] = path[0];

    free(temp);
    free(visited);

}

/**
 * Compare two tours based on their costs for sorting
 * @param a pointer to the first tour
 * @param b pointer to the second tour
 * @return negative if a < b, positive if a > b, zero if equal
 */
int compare_tours(const void* a, const void* b) {
    const tour* t1 = (const tour*)a;
    const tour* t2 = (const tour*)b;
    if (t1->cost < t2->cost - EPS_COST) return -1;
    if (t1->cost > t2->cost + EPS_COST) return 1;
    return 0;
}

/**
 * Mutate a tour by swapping two random nodes (except the first and last nodes)
 * @param path array representing the tour path
 * @param n number of nodes in the tour
 */
void mutate(int* path, int n) {
    int i = 1 + rand() % (n - 1); 
    int j = 1 + rand() % (n - 1); 
    while (j == i) j = 1 + rand() % (n - 1);

    int temp = path[i];
    path[i] = path[j];
    path[j] = temp;

    path[n] = path[0]; 
}

/**
 * Genetic Algorithm to solve the TSP problem
 * @param inst instance containing the problem data
 * @param best_solution pointer to the best solution found
 * @param pop_size size of the population
 * @param timelimit time limit for the algorithm in seconds
 * @return zero on success, non-zero on failures
 */
int genetic_algorithm(instance* inst, tour* best_solution, int pop_size, double timelimit) {

    tour* population = malloc(pop_size * sizeof(tour));
    for (int i = 0; i < pop_size; i++) {
        population[i].path = malloc(inst->nnodes * sizeof(int));
    }

    initialize_population(population, pop_size, inst);

    while (true) {

        double elapsed = second() - inst->tstart;
        if (elapsed > timelimit) {
            if (VERBOSE >= 100) { printf("Time limit of Genetic Algorithm reached\n"); }
            break;
        }

        double ratio = elapsed / timelimit;
        if (ratio > 1.0) ratio = 1.0;

        double p_start = 0.3;
        double p_end = 0.05;
        double p_mutation = p_start + (p_end - p_start) * ratio;

        tour* new_population = malloc(pop_size * sizeof(tour));
        for (int i = 0; i < pop_size; i++) {
            new_population[i].path = malloc((inst->nnodes + 1) * sizeof(int));
        }

        qsort(population, pop_size, sizeof(tour), compare_tours);

        int n_elite = (int)ceil(pop_size * 0.05);
        if (n_elite < 1) n_elite = 1;

        for (int i = 0; i < n_elite; i++) {
            memcpy(new_population[i].path, population[i].path, (inst->nnodes + 1) * sizeof(int));
            new_population[i].cost = population[i].cost;

            if (new_population[i].cost < best_solution->cost - EPS_COST) {
                memcpy(best_solution->path, new_population[i].path, (inst->nnodes + 1) * sizeof(int));
                best_solution->cost = new_population[i].cost;
            }
        }

        for (int i = n_elite; i < pop_size; i++) {

            tour* parent1 = tournament_selection(population, pop_size);
            tour* parent2 = tournament_selection(population, pop_size);

            crossover(parent1->path, parent2->path, new_population[i].path, inst->nnodes);

            repair_solution(new_population[i].path, inst);

            if ((rand() / (double)RAND_MAX) < p_mutation) {
                mutate(new_population[i].path, inst->nnodes);
            }

            two_opt(&new_population[i], inst);

            compute_solution_cost(&new_population[i], inst);

            if (new_population[i].cost < best_solution->cost - EPS_COST) {
                memcpy(best_solution->path, new_population[i].path, (inst->nnodes + 1) * sizeof(int));
                best_solution->cost = new_population[i].cost;
                save_history_incumbent(best_solution->cost, "../data/history_incumbent.txt");
            }
            
        }

        for (int i = 0; i < pop_size; i++) {
            free(population[i].path);
            population[i] = new_population[i];
        }
        free(new_population);

    }

    for (int i = 0; i < pop_size; i++) {
        free(population[i].path);
    }
    free(population);

    plot_history_cost("../data/history_incumbent.txt", "../data/history_incumbent.png");

    return 0;

}