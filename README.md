# OR2_TSP

## Project Structure
```
OR2_TSP/ 
│-- data/ # Directory for dataset files  
│-- include/ # Header files 
│-- src/ # Source code files 
│-- build/ # Build directory (created during compilation) 
│-- CMakeLists.txt # CMake build configuration 
│-- .gitignore # Git ignore file 
│-- README.md # Project documentation
```

## Requirements
* [Gnuplot](http://www.gnuplot.info/):  a portable command-line driven graphing utility
* [Concorde](https://www.math.uwaterloo.ca/tsp/concorde/index.html): TSP solver
* [IBM ILOG CPLEX](https://www.ibm.com/products/ilog-cplex-optimization-studio): IBM CPLEX Optimizer

## Installation
* Clone the repository: `git clone https://github.com/francescostellaa/OR2_TSP.git`
* Create *build* directory: `mkdir build`
* Navigate to the *build* directory: `cd build`
* Run command `cmake ..` to configure the project
* Execute `make` command to build the project

## Command line usage
If you have a file from TSPLIB, use the following command line:
```./tsp -file ../data/<filename> -time_limit <tl> -seed <seed> -alg <algorithm>```

Otherwise, if we want to use artificial data randomly generated, use the following one:
```./tsp -time_limit <tl> -seed <seed> -nodes <nnodes> -alg <algorithm>```

If both -file and -nodes flag are used, it considers input file data.

To see the list of all available algorithms and modes, use the following command line:
```./tsp -help```
