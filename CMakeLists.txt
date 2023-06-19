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

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${LANELET2_INCLUDE_DIRS})
include_directories(include)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${LANELET2_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

# Add the sub.cpp file as a library
add_library(ConverterLib "src/converter_from_llh.cpp" "src/converter_to_llh.cpp")

# Create two executables
add_executable(PCDConverter "src/pcd_conversion.cpp")
add_executable(Lanelet2Converter "src/lanelet_conversion.cpp")

# Link the libraries
target_link_libraries(PCDConverter ConverterLib ${PCL_LIBRARIES} yaml-cpp ${GDAL_LIBRARY} ${GeographicLib_LIBRARIES})
target_link_libraries(Lanelet2Converter ConverterLib ${PCL_LIBRARIES} yaml-cpp ${GDAL_LIBRARY} ${GeographicLib_LIBRARIES})