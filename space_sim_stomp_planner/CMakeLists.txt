cmake_minimum_required(VERSION 3.22)
project(space_sim_stomp_planner)
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_core REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stomp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rsl REQUIRED)
find_package(pluginlib REQUIRED)
# Jazzy's ament dependency processing can flatten STOMP's multiarch library
# location to lib/libstomp.so. Resolve the installed library explicitly.
find_library(SPACE_SIM_STOMP_LIBRARY NAMES stomp
  HINTS ${stomp_LIBRARY_DIRS} ${stomp_LIBRARY_DIRS}/${CMAKE_LIBRARY_ARCHITECTURE}
  REQUIRED)
set_target_properties(stomp::stomp PROPERTIES
  IMPORTED_LOCATION "${SPACE_SIM_STOMP_LIBRARY}"
  IMPORTED_LOCATION_NONE "${SPACE_SIM_STOMP_LIBRARY}")
generate_parameter_library(stomp_moveit_parameters res/stomp_moveit.yaml)
add_library(space_sim_stomp_plugin SHARED src/stomp_moveit_planner_plugin.cpp
  src/stomp_moveit_planning_context.cpp)
target_include_directories(space_sim_stomp_plugin PRIVATE include)
target_compile_features(space_sim_stomp_plugin PUBLIC cxx_std_17)
ament_target_dependencies(space_sim_stomp_plugin moveit_core std_msgs tf2_eigen visualization_msgs)
target_link_libraries(space_sim_stomp_plugin stomp::stomp stomp_moveit_parameters rsl::rsl)
pluginlib_export_plugin_description_file(moveit_core stomp_moveit_plugin_description.xml)
install(TARGETS space_sim_stomp_plugin
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_constraint_cost test/test_constraint_cost.cpp)
  target_include_directories(test_constraint_cost PRIVATE include)
  ament_target_dependencies(test_constraint_cost moveit_core)
  target_link_libraries(test_constraint_cost stomp::stomp rsl::rsl)
endif()
ament_package()
