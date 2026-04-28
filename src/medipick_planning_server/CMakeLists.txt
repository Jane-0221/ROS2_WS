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
  scripts/gazebo_box_target_publisher.py
  scripts/gazebo_trial_initial_pose_manager.py
  scripts/gazebo_cmd_vel_bridge.py
  scripts/image_stamp_republisher.py
  scripts/camera_info_stamp_republisher.py
  scripts/gazebo_discrete_lift_controller.py
  scripts/gazebo_ground_truth_localization.py
  scripts/gazebo_map_publisher.py
  scripts/latched_occupancy_grid_republisher.py
  scripts/gazebo_pre_navigation_manager.py
  scripts/nav2_target_navigator.py
  scripts/slam_loop_validator.py
  scripts/rtabmap_param_tuner.py
  scripts/pick_task_manager.py
  scripts/gazebo_omni_base_driver.py
  scripts/gazebo_mapping_route_runner.py
  scripts/frontier_explorer.py
  scripts/manual_mapping_posture_keeper.py
  scripts/mapping_health_monitor.py
  scripts/mapping_keyboard_teleop.py
  scripts/nav2_activation_gate.py
  scripts/bool_topic_gate.py
  scripts/post_pick_delivery_manager.py
  scripts/medicine_memory_registry.py
  scripts/medicine_memory_target_publisher.py
  scripts/medicine_memory_gazebo_recorder.py
  scripts/medicine_memory_cli.py
  scripts/fetch_medicine_coordinator.py
  scripts/imu_ready_relay.py
  scripts/medicine_location_registry.py
  scripts/wait_for_tf_ready.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/medicine_memory_store.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
