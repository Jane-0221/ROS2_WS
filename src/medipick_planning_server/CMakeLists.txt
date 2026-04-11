cmake_minimum_required(VERSION 3.8)
project(medipick_planning_server)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/planning_server.py
  scripts/plan_to_pose_client.py
  scripts/pick_task_flow.py
  scripts/pick_task_runtime.py
  scripts/pick_task_services.py
  scripts/pick_task_shared.py
  scripts/pick_task_utils.py
  scripts/mock_vision_publisher.py
  scripts/mock_initial_pose_manager.py
  scripts/mock_pick_path_publisher.py
  scripts/pick_task_manager.py
  scripts/medicine_location_registry.py
  scripts/fetch_medicine_coordinator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
