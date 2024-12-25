cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 11)
project(test_clustering)
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED )
set(EXTERNAL_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${OpenCV_LIBS} ${Boost_LIBRARIES})
add_executable(test_clustering
               src/test.cpp src/obstacles.cpp src/ultis.cpp

)
include_directories(
    ${EXTERNAL_INCLUDE_DIRS}
    include
)
target_link_libraries(test_clustering ${LIBRARIES} ${EXTERNAL_LIBS})
