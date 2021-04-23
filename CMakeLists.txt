cmake_minimum_required(VERSION 3.4.1)

project(TransForms)

find_package(Eigen3 REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    include
)
add_executable(${PROJECT_NAME} 
    main.cpp
    TransForms.cpp
)
target_link_libraries(${PROJECT_NAME} 
    pthread
)
