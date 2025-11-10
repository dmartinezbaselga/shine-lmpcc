cmake_minimum_required(VERSION 2.8.3)
project(lmpcc_tools)

add_compile_options(-std=c++11)
add_compile_options(-fstack-protector)
set(CMAKE_CXX_FLAGS "-O3 -Wall -DCMAKE_BUILD_TYPE=Release")

find_package(catkin REQUIRED COMPONENTS
  # eigen_conversions
  # geometry_msgs
  # nav_msgs
  # roscpp
  # roslint
  # roslib
  # sensor_msgs
  # std_msgs
  # tf
  # tf_conversions
  # trajectory_msgs
  # visualization_msgs
  # lmpcc_msgs
  eigen_conversions
  lmpcc_msgs
)

find_package(Eigen3 REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS lmpcc_msgs eigen_conversions#eigen_conversions geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf tf_conversions visualization_msgs lmpcc_msgs roslib
  INCLUDE_DIRS include include/${PROJECT_NAME}
  LIBRARIES lmpcc_tools
)

catkin_install_python(PROGRAMS nodes/goal_publishing_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(include)

# ## BUILD ###
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)

add_library(lmpcc_tools
  src/collision_region.cpp
)

add_dependencies(lmpcc_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Test Case ####
install(
  DIRECTORY include/lmpcc_tools/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# add_executable(risk_computation_node
#   scripts/risk-eval-cpp/risk_computation_node.cpp
# )

# target_link_libraries(risk_computation_node
#   lmpcc_tools
#   ${catkin_LIBRARIES}
# )

# add_dependencies(risk_computation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})