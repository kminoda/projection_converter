cmake_minimum_required(VERSION 3.1)

# Define the project and set the C++ standard
project(ProjectionConverter)
set(CMAKE_CXX_STANDARD 14)

# Find required packages
find_package(PCL 1.3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GDAL REQUIRED)
# find_package(GeographicLib REQUIRED)
find_path(GeographicLib_INCLUDE_DIRS NAMES GeographicLib/Config.h)
find_library(GeographicLib_LIBRARIES NAMES Geographic)

include_directories(${PCL_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} ${GDAL_INCLUDE_DIR} ${GeographicLib_INCLUDE_DIRS} include)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

# Add all cpp files in the src directory
file(GLOB SOURCES "src/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCES})

# Link the libraries
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} yaml-cpp ${GDAL_LIBRARY} ${GeographicLib_LIBRARIES})
