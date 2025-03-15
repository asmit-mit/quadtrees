cmake_minimum_required(VERSION 3.0.2)
project(quadtrees)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Find required catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp)

# Declare catkin package
catkin_package(INCLUDE_DIRS include LIBRARIES quadtrees_lib CATKIN_DEPENDS
               roscpp)

# Specify additional include directories
include_directories(include ${catkin_INCLUDE_DIRS})

# Build the QuadTree library
add_library(quadtrees_lib src/QuadTree.cpp)

# Link the QuadTree library with catkin libraries
target_link_libraries(quadtrees_lib ${catkin_LIBRARIES})

# Install the library and headers
install(
  TARGETS quadtrees_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h")

# Example node using the library
add_executable(quad_trees_visualizer_node src/quad_trees_visualizer_node.cpp)

# Link the example node with the QuadTree library
target_link_libraries(quad_trees_visualizer_node ${catkin_LIBRARIES}
                      quadtrees_lib)

# Install the example node
install(TARGETS quad_trees_visualizer_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
