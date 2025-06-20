cmake_minimum_required(VERSION 3.10.0)
project(tsp VERSION 0.1.0 LANGUAGES C)

# find_package(PackageName REQUIRED)

include_directories(/opt/ibm/ILOG/CPLEX_Studio2211/cplex/include)


# header files
include_directories(include)
include_directories(${CPLEX_PATH}/include/ilcplex)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(CPLEXDIR /Applications/CPLEX_Studio2211/cplex)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${CPLEXDIR}/include/ilcplex")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L${CPLEXDIR}/lib/arm64_osx/static_pic -L. -lcplex -lpthread -ldl")

# Find and link pthreads
find_package(Threads REQUIRED)

# Add source files
add_executable(tsp
    src/main.c
    src/tsp_utilities.c
    src/chrono.c
    src/greedy.c
    src/k_opt.c
    src/vns.c
    src/grasp.c
    src/tabu.c
    src/cplex_model.c
    src/cplex_utilities.c
    src/mincut.c
    src/matheuristics.c
    src/genetic.c
    src/extra_mileage.c
)

target_link_libraries(tsp PRIVATE ${CPLEX_LIBRARIES} dl m)
target_link_libraries(tsp PRIVATE Threads::Threads)

include(CPack)