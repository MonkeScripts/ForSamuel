cmake_minimum_required(VERSION 3.12.2)
project(bluerov_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch config mavros_params models urdf
  DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/ground_truth_to_mavros.py
  scripts/dvl_to_mavros.py
  scripts/bluerov_movement.py
  scripts/world_objects_to_markers.py
  scripts/locomotion_action_server.py
  scripts/bluerov_square_mission_tree.py
  scripts/node_registry.py
  scripts/arm_and_set_mode.py
  scripts/goto.py
  DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks(hooks/setup.dsv)

ament_package()
