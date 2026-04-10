cmake_minimum_required(VERSION 3.8)
project(medipick_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/default_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
