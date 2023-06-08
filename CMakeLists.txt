cmake_minimum_required(VERSION 3.0.0)
project(DiscretePointsSmoother VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development NumPy)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Protobuf REQUIRED)

if (Protobuf_FOUND)  # 需要特别注意Protobuf_FOUND的大小写
    INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIR})
else (Protobuf_FOUND)
    MESSAGE(FATAL_ERROR "PROTOBUF NOT FOUNED")
endif (Protobuf_FOUND)

include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(${Boost_INCLUDE_DIRS})

link_directories(${PROTOBUF_LIBRARIES})

add_subdirectory(abseil-cpp)

FILE(GLOB_RECURSE SRC_LIST "proto/*.cc" "util/*.cc" "piecewise_jerk/*.cc" "src/*.cc")

add_executable(${PROJECT_NAME} ${SOURCES} ${ALL_SOURCES} ${ALL_HEADERS}
                               ${SRC_LIST}
                               src/main.cpp
                               )

target_link_libraries(${PROJECT_NAME} osqp::osqp
                                      OsqpEigen::OsqpEigen
                                      Python3::Python 
                                      Python3::NumPy
                                      ${PROTOBUF_LIBRARY_DEBUG})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} libipopt.so libadolc.so)
target_link_libraries(${PROJECT_NAME} absl::base absl::strings absl::str_format)

target_include_directories(${PROJECT_NAME} PUBLIC  include proto util ${PROJECT_SOURCE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${Python3_INCLUDE_DIRS} 
                                                   ${Python3_NumPy_INCLUDE_DIRS})
