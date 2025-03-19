cmake_minimum_required(VERSION 3.10.0)
project(tsp VERSION 0.1.0 LANGUAGES C)

# find_package(PackageName REQUIRED)

include_directories(/opt/ibm/ILOG/CPLEX_Studio2211/cplex/include)
# find_package(Boost COMPONENTS filesystem REQUIRED)
# include_directories(${PackageName_INCLUDE_DIRS} )
# include_directories(${Boost_INCLUDE_DIRS})

# Add the include directory containing vrp.h
# header files
include_directories(include)
include_directories(${CPLEX_PATH}/include/ilcplex)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(CPLEXDIR /Applications/CPLEX_Studio2211/cplex)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${CPLEXDIR}/include/ilcplex")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L${CPLEXDIR}/lib/arm64_osx/static_pic -L. -lcplex -lpthread -ldl")

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
        include/tabu.h
)

# target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY})
target_link_libraries(tsp PRIVATE ${CPLEX_LIBRARIES} dl m)

include(CPack)