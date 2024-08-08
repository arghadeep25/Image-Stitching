cmake_minimum_required(VERSION 3.5)
project(image_stitching VERSION 0.1 DESCRIPTION "Image Stitching" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DPROJECT_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\")

# Set build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Find OpenCV
find_package(OpenCV REQUIRED)
find_package(OpenCV COMPONENTS core highgui imgproc features2d REQUIRED)

# Include OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

# Source and include directories
SET(SRC_DIR src)
SET(INCLUDE_DIR include)

# Glob for source and header files
FILE(GLOB SRC_FILES ${SRC_DIR}/*.cpp)
FILE(GLOB INCLUDE_FILES ${INCLUDE_DIR}/*.hpp)

# Main executable
# add_executable(image_stitching ${SRC_FILES} ${INCLUDE_FILES})
# target_include_directories(image_stitching PUBLIC ${INCLUDE_DIR})
# target_link_libraries(image_stitching SHARED ${OpenCV_LIBRARIES} pthread)

# Create a library from your source files
add_library(image_stitching_lib ${SRC_FILES} ${INCLUDE_FILES})
target_include_directories(image_stitching_lib PUBLIC ${INCLUDE_DIR})
target_link_libraries(image_stitching_lib ${OpenCV_LIBRARIES})

# Create the main executable
add_executable(image_stitching ${SRC_FILES} ${INCLUDE_FILES})  # Ensure you have a main.cpp for the executable
target_link_libraries(image_stitching image_stitching_lib)

# Add Google Test
add_subdirectory(tests)