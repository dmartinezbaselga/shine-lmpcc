cmake_minimum_required(VERSION 3.0.2)
project(lmpcc_solver)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  message(WARNING "Compiling with OPENMP")
endif()

set(SYSTEM_TO_USE "Jackal")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  std_msgs
  std_srvs
  message_generation
  sensor_msgs
  rospy
  lmpcc_tools
  ros_tools
)

catkin_python_setup()

add_service_files(
  DIRECTORY
  srv
  FILES
  PropagateDynamics.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS std_msgs rospy std_srvs message_generation
  INCLUDE_DIRS include include/${PROJECT_NAME} scripts/${SYSTEM_TO_USE}FORCESNLPsolver/include
  LIBRARIES lmpcc_solver
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  scripts/${SYSTEM_TO_USE}FORCESNLPsolver/include
)

# if(EXISTS ${PROJECT_SOURCE_DIR}/scripts/${SYSTEM_TO_USE}FORCESNLPsolver/${SYSTEM_TO_USE}FORCESNLPsolver_interface.c)

add_library(lmpcc_solver
  src/${SYSTEM_TO_USE}Solver.cpp
  scripts/${SYSTEM_TO_USE}FORCESNLPsolver/${SYSTEM_TO_USE}FORCESNLPsolver_interface.c
  scripts/${SYSTEM_TO_USE}FORCESNLPsolver/${SYSTEM_TO_USE}FORCESNLPsolver_model.c
)
# else()
#   message(SEND_ERROR "SOLVER IS MISSING. PLEASE GENERATE A SOLVER FIRST (see 02_generate_solver.sh). path checked = ${PROJECT_SOURCE_DIR}/scripts/${SYSTEM_TO_USE}FORCESNLPsolver/${SYSTEM_TO_USE}FORCESNLPsolver_interface.c")
# endif()

add_dependencies(lmpcc_solver 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(lmpcc_solver
  ${PROJECT_SOURCE_DIR}/scripts/${SYSTEM_TO_USE}FORCESNLPsolver/lib/lib${SYSTEM_TO_USE}FORCESNLPsolver.so
)


# Install lmpcc library
install(TARGETS lmpcc_solver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/online_dynamics_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
