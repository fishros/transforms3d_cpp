cmake_minimum_required(VERSION 3.4.1)
project(transforms3d_cpp)
set(LIB_VERSION 0.1.0)
set(LIB_NAME  transforms3d_cpp)
SET(CMAKE_BUILD_TYPE "DEBUG")

# add_compile_options(-Werror)
find_package(Eigen3 REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    include
)

# 添加代码文件
file(GLOB_RECURSE PROJECT_SRC "src/*.cpp")
# 添加测试文件
file(GLOB_RECURSE PROJECT_TESTS "tests/*_test.cpp")


add_library(${LIB_NAME}
    SHARED
    ${PROJECT_SRC}
)
target_link_libraries(${LIB_NAME} ${EIGEN3_LIB})


if(${CMAKE_BUILD_TYPE} MATCHES "DEBUG")

find_package(GTest REQUIRED)
if(NOT GTest_FOUND)
    message("NOT found GTest")
endif()
# 添加测试可执行文件
add_executable(${PROJECT_NAME}_test  
    ${PROJECT_SRC}
    ${PROJECT_TESTS}
)
target_link_libraries(${PROJECT_NAME}_test
    gtest
)

# 代码格式检查
set(CPPLINT_SRC ${FISH_PROTOCOL_SRC} ${FISH_PROTOCOL_TESTS})
list(REMOVE_DUPLICATES CPPLINT_SRC)
foreach(v ${CPPLINT_SRC})
  execute_process(
    COMMAND cpplint --quiet ${v}
    ERROR_VARIABLE _result
  )
  if( _result)
    message(FATAL "cpplint check failed!  \n ${_result}")
  endif()
endforeach(v)

endif()


# 生成配置文件，安装到系统
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${LIB_NAME}
  EXPORT ${LIB_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# add lib.config
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        ${LIB_NAME}Config.cmake
        VERSION 1.0
        COMPATIBILITY AnyNewerVersion 
        )

install(EXPORT ${LIB_NAME}
        FILE ${LIB_NAME}Config.cmake
        DESTINATION lib/cmake/${LIB_NAME}
        )
