cmake_minimum_required(VERSION 3.12.2)
project(bluerov_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch config mavros_params models
  DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/ground_truth_to_mavros.py
  scripts/dvl_to_mavros.py
  scripts/bluerov_movement.py
  DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks(hooks/setup.dsv)

ament_package()
