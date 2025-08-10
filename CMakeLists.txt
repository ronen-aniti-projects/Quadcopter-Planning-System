cmake_minimum_required(VERSION 3.10.0)
project(Quadcopter_Planning_System VERSION 0.1.0 LANGUAGES C CXX)

add_compile_options(-Wall -pedantic-errors)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Fetch Eigen 
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
)

# Make Eigen available
FetchContent_MakeAvailable(eigen)

# Fetch KDTree library
FetchContent_Declare(
    KDTree
    GIT_REPOSITORY https://github.com/crvs/KDTree
    
)

# Make KDTree available
FetchContent_MakeAvailable(KDTree)

# Build the KDTree as its own library target
add_library(KDTreeLib STATIC
    ${KDTree_SOURCE_DIR}/KDTree.cpp)

# Include the .hpp file for the KDTree library
target_include_directories(KDTreeLib PUBLIC ${KDTree_SOURCE_DIR})

# Main Executable 
add_executable(main_cpp src/main.cpp 
               src/global_planning.cpp 
               src/local_planning.cpp 
               src/obstacle_processor.cpp
               src/trajectory_planning.cpp)

# Include headers
target_include_directories(main_cpp PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Link Eigen and KDTree
target_link_libraries(main_cpp PRIVATE Eigen3::Eigen KDTreeLib)