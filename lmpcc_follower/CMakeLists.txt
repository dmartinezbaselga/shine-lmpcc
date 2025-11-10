cmake_minimum_required(VERSION 2.8.3)
project(lmpcc_follower)

add_compile_options(-fstack-protector)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

set(CONFIGURATION_TO_USE "SimpleSim")
set(SYSTEM_TO_USE "Prius")
string(TOLOWER ${CONFIGURATION_TO_USE} LOWER_CONFIGURATION_TO_USE)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    eigen_conversions
    std_msgs
    dynamic_reconfigure
    lmpcc_msgs
    lmpcc_tools
    lmpcc
)

# find_package(Boost REQUIRED COMPONENTS filesystem thread)

add_definitions(${EIGEN_DEFINITIONS})

generate_dynamic_reconfigure_options(cfg/${LOWER_CONFIGURATION_TO_USE}.cfg)

catkin_package(
  CATKIN_DEPENDS eigen_conversions roscpp std_msgs lmpcc_msgs lmpcc_tools lmpcc
  INCLUDE_DIRS include/${PROJECT_NAME}
  LIBRARIES lmpcc_follower
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME} # Include sources
  include/solvers # Forces Pro
)

# Follower Library
add_library(lmpcc_follower
  src/configuration.cpp
  src/follower.cpp
  src/pid_stanley.cpp
  include/${PROJECT_NAME}/generated_cpp/${SYSTEM_TO_USE}Solver.cpp
)
add_dependencies(lmpcc_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# The follower node
add_executable(follower_node src/follower_node.cpp
  ${PROJECT_SOURCE_DIR}/include/solvers/${SYSTEM_TO_USE}FORCESNLPsolver/${SYSTEM_TO_USE}FORCESNLPsolver_interface.c
  ${PROJECT_SOURCE_DIR}/include/solvers/${SYSTEM_TO_USE}FORCESNLPsolver/${SYSTEM_TO_USE}FORCESNLPsolver_model.c
)
target_link_libraries(follower_node
  lmpcc_follower
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/include/solvers/${SYSTEM_TO_USE}FORCESNLPsolver/lib/lib${SYSTEM_TO_USE}FORCESNLPsolver.so
)
add_dependencies(follower_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_definitions(follower_node PUBLIC -DSYSTEM_${CONFIGURATION_TO_USE})
add_definitions(-DSYSTEM_${CONFIGURATION_TO_USE})

install(
  TARGETS follower_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

### Test Case ####
# install(
#   DIRECTORY include/lmpcc_follower/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )