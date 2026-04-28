cmake_minimum_required(VERSION 3.8)
project(medipick_simple3_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes models rviz urdf vendor worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/simple5_asset_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
