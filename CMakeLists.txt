cmake_minimum_required(VERSION 2.8)

project(ARM_VO)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O3 -std=c++11 -funroll-loops -Ofast")

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found.")
endif()


if(NOT TARGET OpenMP::OpenMP_CXX)
    add_library(OpenMP_TARGET INTERFACE)
    add_library(OpenMP::OpenMP_CXX ALIAS OpenMP_TARGET)
    target_compile_options(OpenMP_TARGET INTERFACE ${OpenMP_CXX_FLAGS})
    find_package(Threads REQUIRED)
    target_link_libraries(OpenMP_TARGET INTERFACE Threads::Threads)
    target_link_libraries(OpenMP_TARGET INTERFACE ${OpenMP_CXX_FLAGS})
endif()

# find_package(OpenMP REQUIRED)
# if(OpenMP_FOUND)
# 	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
# else()
#     message(WARNING "OpenMP not found.")
# endif()

include_directories(include)

file(GLOB SOURCES "src/*.cpp")
add_executable(ARM_VO ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
