CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(ORBqin)
SET(CMAKE_BUILD_TYPE Debug)
LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11  -Wall  -g -O0 -march=native" )

FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)


SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    src/ORBextractor.cpp
    src/Frame.cpp
    src/MapPoint.cpp
    src/ORBmatcher.cpp
    src/System.cpp
    src/Tracker.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${EIGEN3_LIBS}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR/tests})

add_executable(testFirst
    tests/TestFirst.cpp)
target_link_libraries(testFirst ${PROJECT_NAME})


