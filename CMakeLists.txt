cmake_minimum_required(VERSION 3.0)
# set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(test_clustering)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/admin11/projects/g2o/cmake_modules/)

find_package(OpenCV REQUIRED )
find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package(SuiteSparse REQUIRED)
find_package(G2O REQUIRED)
find_package(fmt REQUIRED)
set(G2O_LIBS
    g2o_core
    g2o_stuff
    g2o_types_slam2d
    g2o_types_slam3d
    g2o_solver_dense
    g2o_solver_eigen
)
# Set external include directories
set(EXTERNAL_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS} ${fmt_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${OpenCV_LIBS} ${SUITESPARSE_LIBRARIES} ${G2O_LIBS} ${Boost_LIBRARIES} fmt::fmt)
add_executable(test_clustering
               src/test.cpp src/obstacles.cpp src/ultis.cpp

)
# add_executable(test_planner 
#                src/test_planner.cpp 
#                src/obstacles.cpp 
#                src/ultis.cpp
#                src/homotopy_class_planner/homotopy_class_planner.cpp
#                src/homotopy_class_planner/graph_search.cpp
#                src/homotopy_class_planner/timed_elastic_band.cpp
#                src/optimization/optimal_planner.cpp
#                src/global_planner/cubic_spline.cpp 
#                src/global_planner/rrt_star.cpp 

# )      
add_executable(
    test_g2o src/test_g2o.cpp
)
include_directories(
    ${EXTERNAL_INCLUDE_DIRS}
    include
    include/homotopy_class_planner
    include/g2o_types
    include/global_planner
    include/optimization
)
target_link_libraries(test_clustering ${LIBRARIES} ${EXTERNAL_LIBS})
target_link_libraries(test_g2o ${LIBRARIES} ${EXTERNAL_LIBS})
# target_link_libraries(test_planner ${LIBRARIES} ${EXTERNAL_LIBS})
# cmake_minimum_required(VERSION 3.0)
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# project(test_clustering)
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/admin11/projects/g2o/cmake_modules/)
# # Find required packages
# find_package(OpenCV REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system thread graph)
# find_package(Eigen3 REQUIRED)
# find_package(SuiteSparse REQUIRED)
# find_package(G2O REQUIRED)
# find_package(fmt REQUIRED) 

# # Set external include directories
# include_directories(
#     ${OpenCV_INCLUDE_DIRS}
#     ${EIGEN3_INCLUDE_DIRS}
#     ${SUITESPARSE_INCLUDE_DIRS}
#     ${G2O_INCLUDE_DIRS}
#     ${fmt_INCLUDE_DIRS}
#     include
#     include/homotopy_class_planner
#     include/g2o_types
#     include/global_planner
#     include/optimization
# )

# # Specify G2O components needed
# set(G2O_LIBS
#     g2o_core
#     g2o_stuff
#     g2o_types_slam2d
#     g2o_types_slam3d
#     g2o_solver_dense
#     g2o_solver_eigen
# )

# # Set external libraries
# set(EXTERNAL_LIBS
#     ${OpenCV_LIBS}
#     ${SUITESPARSE_LIBRARIES}
#     ${G2O_LIBS}
#     ${Boost_LIBRARIES}
#     fmt::fmt
# )

# # Add executables
# add_executable(test_clustering
#     src/test.cpp
#     src/obstacles.cpp
#     src/ultis.cpp
# )

# add_executable(test_g2o
#     src/test_g2o.cpp
# )

# # Link libraries
# target_link_libraries(test_clustering ${EXTERNAL_LIBS})
# target_link_libraries(test_g2o ${EXTERNAL_LIBS})