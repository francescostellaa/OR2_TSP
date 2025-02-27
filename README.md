# OR2_TSP

## Project Structure

## Requirements
* [Gnuplot](http://www.gnuplot.info/):  a portable command-line driven graphing utility
* [CPLEX](https://www.ibm.com/it-it/analytics/cplex-optimizer): IBM CPLEX Optimizer
* [Concorde](https://www.math.uwaterloo.ca/tsp/concorde/index.html): TSP solver

## Installation
* Clone the repository: `git clone https://github.com/francescostellaa/OR2_TSP.git`
* Create *build* directory: `mkdir build`
* Navigate to the *build* directory: `cd build`
* Run command `cmake ..` to configure the project
* Execute `make` command to build the project

## Command line usage
If we have a file from TSPLIB, use the following command line:
```./tsp -file ../data/<filename> -time_limit <tl> -seed <seed>```

Otherwise, if we want to use artificial data randomly generated, use the following one:
```./tsp -time_limit <tl> -seed <seed> -nodes <nnodes>```

If both -file and -nodes flag are used, it considers input file data.

