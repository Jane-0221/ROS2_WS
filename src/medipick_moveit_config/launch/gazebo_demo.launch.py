from pathlib import Path
import os
import re

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name: str, relative_path: str) -> str:
    package_path = Path(get_package_share_directory(package_name))
    return (package_path / relative_path).read_text(encoding="utf-8")


def load_yaml(package_name: str, relative_path: str) -> dict:
    package_path = Path(get_package_share_directory(package_name))
    with open(package_path / relative_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def inject_minimal_inertial(urdf: str, link_name: str) -> str:
    pattern = rf'<link\s+name="{re.escape(link_name)}"\s*/>'
    replacement = f"""
  <link name="{link_name}">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="0.001" />
      <inertia ixx="1e-6" ixy="0.0" ixz="0.0" iyy="1e-6" iyz="0.0" izz="1e-6" />
    </inertial>
  </link>"""
    updated_urdf, count = re.subn(pattern, replacement, urdf, count=1)
    if count != 1:
        raise RuntimeError(f"Expected an empty link named '{link_name}' in the Gazebo URDF template")
    return updated_urdf


def rename_joint(urdf: str, old_name: str, new_name: str) -> str:
    pattern = rf'(<joint\s+name="){re.escape(old_name)}(")'
    updated_urdf, count = re.subn(pattern, rf"\1{new_name}\2", urdf, count=1)
    if count == 0:
        return urdf
    return updated_urdf


def set_joint_type(urdf: str, joint_name: str, joint_type: str) -> str:
    pattern = rf'(<joint\s+name="{re.escape(joint_name)}"\s+type=")([^"]+)(")'
    updated_urdf, count = re.subn(pattern, rf"\1{joint_type}\3", urdf, count=1)
    if count == 0:
        return urdf
    return updated_urdf


def add_raise_velocity_command_interface(urdf: str) -> str:
    pattern = (
        r'(<joint name="raise_joint">\s*'
        r'<command_interface name="position" />\s*)'
        r'(<state_interface name="position">\s*'
        r'<param name="initial_value">[^<]+</param>\s*'
        r'</state_interface>\s*'
        r'<state_interface name="velocity" />\s*'
        r'</joint>)'
    )
    replacement = r'\1<command_interface name="velocity" />\n      \2'
    updated_urdf, count = re.subn(pattern, replacement, urdf, count=1)
    if count == 0:
        raise RuntimeError("Expected raise_joint ros2_control block in Gazebo URDF template")
    return updated_urdf


def build_spawner_arguments(
    controller_name: str,
    ros2_controllers_file: Path | None = None,
    controller_type: str | None = None,
) -> list[str]:
    arguments = [
        controller_name,
        "--controller-manager",
        "/controller_manager",
        "--controller-manager-timeout",
        "120.0",
        "--service-call-timeout",
        "120.0",
        "--switch-timeout",
        "120.0",
    ]
    if controller_type:
        arguments.extend(["--controller-type", controller_type])
    if ros2_controllers_file is not None:
        arguments.extend(["--param-file", str(ros2_controllers_file)])
    return arguments


def build_gazebo_robot_description(description_package: str, moveit_package: str) -> str:
    description_share_dir = Path(get_package_share_directory(description_package))
    base_urdf = load_file(description_package, "urdf/simple3_moveit.urdf")
    ros2_controllers_file = (
        Path(get_package_share_directory(moveit_package)) / "config" / "ros2_controllers.yaml"
    )
    gz_ros2_control_file = (
        Path(get_package_share_directory(moveit_package)) / "config" / "gz_ros2_control.yaml"
    )

    if "<plugin>mock_components/GenericSystem</plugin>" not in base_urdf:
        raise RuntimeError("Expected mock ros2_control hardware plugin in simple3_moveit.urdf")

    gazebo_urdf = base_urdf.replace(
        "<plugin>mock_components/GenericSystem</plugin>",
        "<plugin>gz_ros2_control/GazeboSimSystem</plugin>",
        1,
    )
    gazebo_urdf = add_raise_velocity_command_interface(gazebo_urdf)
    gazebo_urdf = gazebo_urdf.replace(
        "</hardware>",
        "    <hold_joints>false</hold_joints>\n"
        "    <position_proportional_gain>5.0</position_proportional_gain>\n"
        "    </hardware>",
        1,
    )

    # Gazebo requires inertials on the dummy floating-base chain links, while MoveIt is fine without them.
    for link_name in ("base_x_link", "base_y_link", "base_theta_link"):
        gazebo_urdf = inject_minimal_inertial(gazebo_urdf, link_name)

    # The exported CAD URDF reuses several child link names as fixed-joint names; Gazebo's frame graph rejects that.
    renamed_joints = {
        "h2_motor": "h2_motor_fixed_joint",
        "l2_motor": "l2_motor_fixed_joint",
        "l3_motor": "l3_motor_fixed_joint",
        "l4_motor": "l4_motor_fixed_joint",
        "l5_motor": "l5_motor_fixed_joint",
        "l6_motor": "l6_motor_fixed_joint",
        "r2_motor": "r2_motor_fixed_joint",
        "r3_motor": "r3_motor_fixed_joint",
        "r4_motor": "r4_motor_fixed_joint",
        "r5_motor": "r5_motor_fixed_joint",
        "r6_motor": "r6_motor_fixed_joint",
    }
    for old_name, new_name in renamed_joints.items():
        gazebo_urdf = rename_joint(gazebo_urdf, old_name, new_name)

    # These auxiliary joints are outside the mobile-arm planning chain; freezing them keeps
    # Gazebo from needing extra controllers and removes incomplete-state warnings in MoveIt.
    for joint_name in ("l1_joint", "l2_joint", "l3_joint", "l4_joint", "l5_joint"):
        gazebo_urdf = set_joint_type(gazebo_urdf, joint_name, "fixed")

    gazebo_urdf = gazebo_urdf.replace(
        f"package://{description_package}/",
        f"file://{description_share_dir.as_posix()}/",
    )

    gazebo_suffix = f"""
  <gazebo reference="base_imu_link">
    <sensor name="base_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <topic>/medipick/imu</topic>
    </sensor>
  </gazebo>
  <gazebo reference="cam_sensor_link">
    <sensor name="head_rgbd" type="rgbd_camera">
      <pose>0 0 0 0 0 0</pose>
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
      <topic>/medipick/depth_camera</topic>
      <camera>
        <horizontal_fov>1.5707963267948966</horizontal_fov>
        <image>
          <width>640</width>
          <height>400</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.10</near>
          <far>10.00</far>
        </clip>
      </camera>
      <depth_camera>
        <clip>
          <near>0.10</near>
          <far>10.00</far>
        </clip>
      </depth_camera>
    </sensor>
  </gazebo>
  <gazebo>
    <plugin
      filename="ignition-gazebo-joint-position-controller-system"
      name="gz::sim::systems::JointPositionController">
      <joint_name>raise_joint</joint_name>
      <topic>/medipick/task/lift_target_height</topic>
      <p_gain>12000.0</p_gain>
      <i_gain>40.0</i_gain>
      <d_gain>250.0</d_gain>
      <i_max>4000.0</i_max>
      <i_min>-4000.0</i_min>
      <cmd_max>30000.0</cmd_max>
      <cmd_min>-30000.0</cmd_min>
    </plugin>
  </gazebo>
  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>{ros2_controllers_file}</parameters>
      <ros>
        <argument>--ros-args</argument>
        <argument>--params-file</argument>
        <argument>{gz_ros2_control_file}</argument>
      </ros>
      <robot_param>robot_description</robot_param>
      <robot_param_node>medipick_robot_state_publisher</robot_param_node>
    </plugin>
  </gazebo>
"""
    return gazebo_urdf.replace("</robot>", gazebo_suffix + "\n</robot>")


def generate_launch_description():
    description_package = "medipick_simple3_description"
    moveit_package = "medipick_moveit_config"

    world_file = str(
        Path(get_package_share_directory(description_package)) / "worlds" / "medipick_test.world.sdf"
    )
    world_file_launch = LaunchConfiguration("world_file")
    rviz_config_file = str(Path(get_package_share_directory(moveit_package)) / "rviz" / "moveit.rviz")
    gz_sim_launch = str(Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py")

    robot_description = {
        "robot_description": build_gazebo_robot_description(description_package, moveit_package),
    }
    robot_description_semantic = {
        "robot_description_semantic": load_file(moveit_package, "config/medipick.srdf"),
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(moveit_package, "config/kinematics.yaml"),
    }
    robot_description_planning = {
        "robot_description_planning": load_yaml(moveit_package, "config/joint_limits.yaml"),
    }

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(load_yaml(moveit_package, "config/ompl_planning.yaml"))
    moveit_controllers = load_yaml(moveit_package, "config/moveit_controllers.yaml")

    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    sensors_3d = load_yaml(moveit_package, "config/sensors_3d.yaml")
    sensor_name = sensors_3d["sensors"][0]
    sensor_config = sensors_3d[sensor_name]
    octomap_sensor_parameters = {
        "sensors": sensors_3d["sensors"],
        f"{sensor_name}.sensor_plugin": sensor_config["sensor_plugin"],
        f"{sensor_name}.point_cloud_topic": "/medipick/depth_camera/points",
        f"{sensor_name}.max_range": sensor_config["max_range"],
        f"{sensor_name}.point_subsample": sensor_config["point_subsample"],
        f"{sensor_name}.padding_offset": sensor_config["padding_offset"],
        f"{sensor_name}.padding_scale": sensor_config["padding_scale"],
        f"{sensor_name}.max_update_rate": sensor_config["max_update_rate"],
        f"{sensor_name}.filtered_cloud_topic": sensor_config["filtered_cloud_topic"],
    }
    octomap_parameters = {
        "octomap_frame": "world",
        "octomap_resolution": 0.03,
        "max_range": 3.0,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    resource_paths = [
        str(Path(get_package_share_directory(description_package))),
        str(Path(get_package_share_directory(description_package)) / "models"),
        str(Path(get_package_share_directory(moveit_package))),
    ]
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_resource_path:
        resource_paths.append(existing_resource_path)

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("gazebo_gui", default_value="false"),
            DeclareLaunchArgument("with_move_group", default_value="true"),
            DeclareLaunchArgument("bridge_pointcloud", default_value="false"),
            DeclareLaunchArgument("world_file", default_value=world_file),
            DeclareLaunchArgument("spawn_delay", default_value="2.0"),
            DeclareLaunchArgument("state_publish_delay", default_value="2.5"),
            DeclareLaunchArgument("robot_spawn_z", default_value="0.03"),
            DeclareLaunchArgument("controller_spawn_delay", default_value="8.0"),
            DeclareLaunchArgument("controller_spawn_stagger", default_value="2.0"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", ":".join(resource_paths)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={
                    "gz_args": [TextSubstitution(text="-r "), world_file_launch]
                }.items(),
                condition=IfCondition(LaunchConfiguration("gazebo_gui")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={
                    "gz_args": [TextSubstitution(text="-r -s "), world_file_launch]
                }.items(),
                condition=UnlessCondition(LaunchConfiguration("gazebo_gui")),
            ),
            TimerAction(
                period=LaunchConfiguration("state_publish_delay"),
                actions=[
                    Node(
                        package=moveit_package,
                        executable="joint_state_stamp_republisher.py",
                        name="medipick_joint_state_stamp_republisher",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "input_topic": "/joint_states",
                                "output_topic": "/medipick/joint_states_sim",
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package=moveit_package,
                        executable="tf_stamp_republisher.py",
                        name="medipick_tf_stamp_republisher",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "input_tf_topic": "/medipick/raw_tf",
                                "input_tf_static_topic": "/medipick/raw_tf_static",
                                "output_tf_topic": "/tf",
                                "output_tf_static_topic": "/tf_static",
                            }
                        ],
                        output="screen",
                    ),
                    Node(
                        package="robot_state_publisher",
                        executable="robot_state_publisher",
                        name="medipick_robot_state_publisher",
                        parameters=[robot_description, {"use_sim_time": True}],
                        remappings=[
                            ("/joint_states", "/medipick/joint_states_sim"),
                            ("/tf", "/medipick/raw_tf"),
                            ("/tf_static", "/medipick/raw_tf_static"),
                        ],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("spawn_delay"),
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        parameters=[robot_description, {"use_sim_time": True}],
                        arguments=[
                            "-param",
                            "robot_description",
                            "-name",
                            "medipick",
                            "-z",
                            LaunchConfiguration("robot_spawn_z"),
                        ],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("controller_spawn_delay"),
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=build_spawner_arguments("joint_state_broadcaster"),
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=PythonExpression(
                    [LaunchConfiguration("controller_spawn_delay"), " + ", LaunchConfiguration("controller_spawn_stagger")]
                ),
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=build_spawner_arguments("imu_sensor_broadcaster"),
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=PythonExpression(
                    [LaunchConfiguration("controller_spawn_delay"), " + 2 * ", LaunchConfiguration("controller_spawn_stagger")]
                ),
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=build_spawner_arguments(
                            "tool_controller",
                            ros2_controllers_file=Path(get_package_share_directory(moveit_package)) / "config" / "ros2_controllers.yaml",
                            controller_type="joint_trajectory_controller/JointTrajectoryController",
                        ),
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=PythonExpression(
                    [LaunchConfiguration("controller_spawn_delay"), " + 3 * ", LaunchConfiguration("controller_spawn_stagger")]
                ),
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=build_spawner_arguments(
                            "head_controller",
                            ros2_controllers_file=Path(get_package_share_directory(moveit_package)) / "config" / "ros2_controllers.yaml",
                            controller_type="joint_trajectory_controller/JointTrajectoryController",
                        ),
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=PythonExpression(
                    [LaunchConfiguration("controller_spawn_delay"), " + 4 * ", LaunchConfiguration("controller_spawn_stagger")]
                ),
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=build_spawner_arguments(
                            "mobile_arm_controller",
                            ros2_controllers_file=Path(get_package_share_directory(moveit_package)) / "config" / "ros2_controllers.yaml",
                            controller_type="joint_trajectory_controller/JointTrajectoryController",
                        ),
                        output="screen",
                    )
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/medipick/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    "/medipick/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/medipick/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/medipick/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    "/medipick/task/lift_target_height@std_msgs/msg/Float64]gz.msgs.Double",
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/medipick/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("bridge_pointcloud")),
            ),
            TimerAction(
                period=LaunchConfiguration("state_publish_delay"),
                actions=[
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="medipick_head_rgbd_frame_bridge",
                        arguments=[
                            "--x",
                            "0.0",
                            "--y",
                            "0.0",
                            "--z",
                            "0.0",
                            "--roll",
                            "-1.5707963267948966",
                            "--pitch",
                            "0.0",
                            "--yaw",
                            "-1.5707963267948966",
                            "--frame-id",
                            "cam_sensor_link",
                            "--child-frame-id",
                            "medipick/h2_link/head_rgbd",
                        ],
                        parameters=[{"use_sim_time": True}],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("state_publish_delay"),
                actions=[
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="medipick_base_imu_frame_bridge",
                        arguments=[
                            "--frame-id",
                            "base_link",
                            "--child-frame-id",
                            "medipick/base_theta_link/base_imu",
                        ],
                        parameters=[{"use_sim_time": True}],
                        output="screen",
                    )
                ],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    ompl_planning_pipeline_config,
                    trajectory_execution,
                    moveit_controllers,
                    octomap_parameters,
                    octomap_sensor_parameters,
                    planning_scene_monitor_parameters,
                    {"use_sim_time": True},
                ],
                condition=IfCondition(LaunchConfiguration("with_move_group")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    ompl_planning_pipeline_config,
                    {"use_sim_time": True},
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
