cmake_minimum_required(VERSION 2.8.3)
project(lmpcc)

add_compile_options(-fstack-protector)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations") #-fsanitize=address")
# set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")

find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  message(WARNING "Compiling with OPENMP")
endif()

set(CONFIGURATION_TO_USE "JackalSimulator")
# set(CONFIGURATION_TO_USE "JackalSocnavbench")
set(SYSTEM_TO_USE "Jackal")
string(TOLOWER ${CONFIGURATION_TO_USE} LOWER_CONFIGURATION_TO_USE)

# ## SYSTEM DEPENDENT DEPENDENCIES ###
if(${CONFIGURATION_TO_USE} STREQUAL "Carla")
  set(SYSTEM_PACKAGES
    carla_msgs
    carla_ackermann_control
    carla_simulation_tools
    carla_waypoint_publisher
    prius_description
    roadmap
    roadmap_msgs
    robot_localization
    # local_polygon
  )
elseif(${CONFIGURATION_TO_USE} STREQUAL "Prius")
  set(SYSTEM_PACKAGES
    roadmap
    roadmap_msgs
    # local_polygon
  )
elseif(${CONFIGURATION_TO_USE} STREQUAL "SimpleSim")
  set(SYSTEM_PACKAGES
    simple_sim
    prius_description
    roadmap
    roadmap_msgs
    # local_polygon
  )
elseif(${CONFIGURATION_TO_USE} STREQUAL "JackalSimulator")
  set(SYSTEM_PACKAGES
    pedestrian_simulator
    robot_localization
    # local_polygon
    mobile_robot_state_publisher
    jackal_gazebo
    roadmap
  )
  elseif(${CONFIGURATION_TO_USE} STREQUAL "JackalSocnavbench")
  set(SYSTEM_PACKAGES
    pedestrian_simulator
    robot_localization
    # local_polygon
    mobile_robot_state_publisher
    jackal_gazebo
    roadmap
  )
elseif(${CONFIGURATION_TO_USE} STREQUAL "Jackal")
  set(SYSTEM_PACKAGES
    pedestrian_simulator
    robot_localization
  )
elseif(${CONFIGURATION_TO_USE} STREQUAL "HovergamesGazebo")
  set(SYSTEM_PACKAGES
    mav_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
  )
else()
  set(SYSTEM_PACKAGES
  )
endif()

message(WARNING "System dependent packages: ${SYSTEM_PACKAGES}")

# add_definitions(${PCL_DEFINITIONS})
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cmake_modules
  control_msgs
  dynamic_reconfigure
  eigen_conversions
  geometry_msgs
  kdl_conversions
  kdl_parser
  nav_msgs
  roscpp
  roslint
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  trajectory_msgs
  urdf
  visualization_msgs
  shape_msgs
  lmpcc_msgs
  pcl_ros
  ros_tools
  lmpcc_tools
  lmpcc_solver
  guidance_planner
  ${SYSTEM_PACKAGES}
)

find_package(Boost REQUIRED COMPONENTS thread filesystem)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
# find_package(local_polygon REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/PredictiveController_${CONFIGURATION_TO_USE}.cfg)

message(WARNING "Predictive configuration: cfg/PredictiveController_${SYSTEM_TO_USE}.cfg")

# # Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  srv
  FILES
  LMPCCReset.srv
)

# Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs
#   geometry_msgs
#   sensor_msgs
#   std_msgs
#   nav_msgs
#   shape_msgs
#   visualization_msgs
#   lmpcc_msgs
# )

catkin_package(
  CATKIN_DEPENDS 
  guidance_planner 
  ros_tools 
  actionlib_msgs 
  dynamic_reconfigure 
  eigen_conversions 
  geometry_msgs 
  kdl_conversions 
  kdl_parser 
  nav_msgs 
  roscpp 
  sensor_msgs 
  std_msgs 
  tf 
  tf_conversions 
  urdf 
  visualization_msgs 
  shape_msgs 
  lmpcc_solver 
  lmpcc_msgs
  DEPENDS Boost
  INCLUDE_DIRS include include/interfaces include/lmpcc include/modules_constraints include/modules_objectives
  LIBRARIES lmpcc_controller lmpcc_controller_on_demand
)

# ## BUILD ###
include_directories(
  include
  include/${PROJECT_NAME}
)
include_directories(
  # ${local_polygon_INCLUDE_DIRS}
  # ${guidance_planner_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  json/include/nlohmann
)

add_library(lmpcc_controller
  src/lmpcc_controller.cpp
  src/lmpcc_configuration.cpp
  src/Clothoid.cpp
  src/spline.cpp
  src/experiment-util.cpp
  src/types.cpp

  src/modules_objectives/mpc_base.cpp
  src/modules_objectives/goal_tracking.cpp
  src/modules_objectives/reference_path.cpp
  src/modules_objectives/guidance_objective.cpp

  src/modules_constraints/guidance_constraints.cpp
  src/modules_constraints/ellipsoidal_constraints.cpp
  src/modules_constraints/gaussian_constraints.cpp
  src/modules_constraints/linearized_constraints.cpp

  src/dynamic_obstacle.cpp
  src/interfaces/interface.cpp
  # src/modules_constraints/scenario_constraints.cpp
  # src/scenario/safe_horizon.cpp
  # src/scenario/smpcc.cpp
  # src/scenario/polygon_search.cpp
  # src/scenario/sampler.cpp
  # src/scenario/json_trajectory_sampler.cpp
  # src/scenario/polygon_search.cpp
  # src/scenario/trajectory_sampler.cpp
  # src/scenario/safety_certifier.cpp
  # src/scenario/gaussian_sampler.cpp
  src/interfaces/${LOWER_CONFIGURATION_TO_USE}_interface.cpp
  include/third_party/optuna.cpp
)

add_library(lmpcc_controller_on_demand
  src/lmpcc_controller_on_demand.cpp
  src/lmpcc_configuration.cpp
  src/Clothoid.cpp
  src/spline.cpp
  src/experiment-util.cpp
  src/types.cpp

  src/modules_objectives/mpc_base.cpp
  src/modules_objectives/goal_tracking.cpp
  src/modules_objectives/reference_path.cpp
  src/modules_objectives/guidance_objective.cpp

  src/modules_constraints/guidance_constraints.cpp
  src/modules_constraints/ellipsoidal_constraints.cpp
  src/modules_constraints/gaussian_constraints.cpp
  src/modules_constraints/linearized_constraints.cpp

  src/dynamic_obstacle.cpp
  src/interfaces/interface.cpp
  # src/modules_constraints/scenario_constraints.cpp
  # src/scenario/safe_horizon.cpp
  # src/scenario/smpcc.cpp
  # src/scenario/polygon_search.cpp
  # src/scenario/sampler.cpp
  # src/scenario/json_trajectory_sampler.cpp
  # src/scenario/polygon_search.cpp
  # src/scenario/trajectory_sampler.cpp
  # src/scenario/safety_certifier.cpp
  # src/scenario/gaussian_sampler.cpp
  src/interfaces/${LOWER_CONFIGURATION_TO_USE}_interface.cpp
  include/third_party/optuna.cpp
)

add_dependencies(lmpcc_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(lmpcc_controller_on_demand ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lmpcc_server
  src/lmpcc_control_server.cpp
)
target_link_libraries(lmpcc_server
  lmpcc_controller_on_demand
  lmpcc_controller
  ${catkin_LIBRARIES}
  ${lmpcc_solver_LIBRARIES}
  ${guidance_planner_LIBRARIES}
)
add_dependencies(lmpcc_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_definitions(lmpcc_server PUBLIC -DSYSTEM_${CONFIGURATION_TO_USE})
add_definitions(-DSYSTEM_${CONFIGURATION_TO_USE})

add_executable(lmpcc_node
  src/lmpcc_control_node.cpp
)
target_link_libraries(lmpcc_node
  lmpcc_controller
  ${catkin_LIBRARIES}
  ${lmpcc_solver_LIBRARIES}
  ${guidance_planner_LIBRARIES}
)
add_dependencies(lmpcc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_definitions(lmpcc_node PUBLIC -DSYSTEM_${CONFIGURATION_TO_USE})
add_definitions(-DSYSTEM_${CONFIGURATION_TO_USE})

add_executable(collision_checker_node src/collision_checker_node.cpp)
target_link_libraries(collision_checker_node  ${catkin_LIBRARIES})
add_dependencies(collision_checker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

############
# Install
############

# Install lmpcc library
install(TARGETS lmpcc_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install controller node
install(
  TARGETS lmpcc_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install lmpcc library
install(TARGETS lmpcc_controller_on_demand
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install controller node
install(
  TARGETS lmpcc_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# ## Test Case ####
install(
  DIRECTORY include/lmpcc/ include/interfaces/ include/modules_constraints/ include/modules_objectives/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
