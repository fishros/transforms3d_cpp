cmake_minimum_required(VERSION 3.4.1)
project(TransForms)
set(LIB_VERSION 0.1.0)
set(LIB_NAME  transforms3d_cpp_${LIB_VERSION})

find_package(Eigen3 REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    include
)

add_library(${LIB_NAME}
    SHARED
    TransForms.cpp
)

add_executable(${PROJECT_NAME} 
    main.cpp
    TransForms.cpp
)


install (TARGETS ${LIB_NAME}
    LIBRARY DESTINATION lib)

