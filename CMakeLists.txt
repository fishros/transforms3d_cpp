cmake_minimum_required(VERSION 3.4.1)
project(TransForms)
set(LIB_VERSION 0.1.0)
set(LIB_NAME  transforms3d_cpp)

find_package(Eigen3 REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    include
)

add_library(${LIB_NAME}
    SHARED
    TransForms.cpp
    TransFormsGroup.cpp
)

add_executable(${PROJECT_NAME} 
    main.cpp
    TransForms.cpp
    TransFormsGroup.cpp
)

install (TARGETS ${LIB_NAME}
    LIBRARY DESTINATION lib)

install(
    DIRECTORY ${CMAKE_SOURCE_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h*")

    
install(TARGETS ${LIB_NAME}
 DESTINATION ${CMAKE_SOURCE_DIR}
)
