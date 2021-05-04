cmake_minimum_required(VERSION 3.4.1)
project(example_handeye)

find_package(Eigen3 REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    /usr/local/include/
)

include_directories(
    /usr/local/lib/
)

add_executable(${PROJECT_NAME} 
example_handeye.cpp
)

target_link_libraries(${PROJECT_NAME}  
    transforms3d_cpp
   )