cmake_minimum_required(VERSION 3.5)
project(image_stitching VERSION 0.1 DESCRIPTION "Image Stitching" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED)
find_package(OpenCV COMPONENTS core highgui imgproc features2d REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
SET(SRC_DIR src)
SET(INCLUDE_DIR include)

FILE(GLOB SRC_FILES ${SRC_DIR}/*.cpp)
FILE(GLOB INCLUDE_FILES ${INCLUDE_DIR}/*.hpp)

add_executable(image_stitching ${SRC_FILES} ${INCLUDE_FILES})
target_include_directories(image_stitching PUBLIC ${INCLUDE_DIR})
target_link_libraries(image_stitching ${OpenCV_LIBRARIES} pthread)