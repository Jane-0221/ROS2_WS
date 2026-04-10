from pathlib import Path
import os
import re

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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


def build_gazebo_robot_description(description_package: str, moveit_package: str) -> str:
    description_share_dir = Path(get_package_share_directory(description_package))
    base_urdf = load_file(description_package, "urdf/simple3_moveit.urdf")
    ros2_controllers_file = (
        Path(get_package_share_directory(moveit_package)) / "config" / "ros2_controllers.yaml"
    )

    if "<plugin>mock_components/GenericSystem</plugin>" not in base_urdf:
        raise RuntimeError("Expected mock ros2_control hardware plugin in simple3_moveit.urdf")

    gazebo_urdf = base_urdf.replace(
        "<plugin>mock_components/GenericSystem</plugin>",
        "<plugin>gz_ros2_control/GazeboSimSystem</plugin>",
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
  <link name="head_camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
      <box size="0.06 0.04 0.04" />
      </geometry>
      <material name="camera_gray">
        <color rgba="0.2 0.2 0.2 1.0" />
      </material>
    </visual>
  </link>
  <joint name="head_camera_joint" type="fixed">
    <origin xyz="0 0.08 0.03" rpy="1.5708 0 0" />
    <parent link="h2_link" />
    <child link="head_camera_link" />
  </joint>
  <gazebo reference="head_camera_link">
    <sensor name="head_rgbd" type="rgbd_camera">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <visualize>true</visualize>
      <topic>/medipick/depth_camera</topic>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.10</near>
          <far>3.00</far>
        </clip>
      </camera>
      <depth_camera>
        <clip>
          <near>0.10</near>
          <far>3.00</far>
        </clip>
      </depth_camera>
    </sensor>
  </gazebo>
  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>{ros2_controllers_file}</parameters>
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
        str(Path(get_package_share_directory(moveit_package))),
    ]
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_resource_path:
        resource_paths.append(existing_resource_path)

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("gazebo_gui", default_value="false"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", ":".join(resource_paths)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={"gz_args": f"-r {world_file}"}.items(),
                condition=IfCondition(LaunchConfiguration("gazebo_gui")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={"gz_args": f"-r -s {world_file}"}.items(),
                condition=UnlessCondition(LaunchConfiguration("gazebo_gui")),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[robot_description, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-name",
                    "medipick",
                    "-z",
                    "0.02",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "mobile_arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-type",
                    "joint_trajectory_controller/JointTrajectoryController",
                    "--param-file",
                    str(Path(get_package_share_directory(moveit_package)) / "config" / "ros2_controllers.yaml"),
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "tool_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-type",
                    "joint_trajectory_controller/JointTrajectoryController",
                    "--param-file",
                    str(Path(get_package_share_directory(moveit_package)) / "config" / "ros2_controllers.yaml"),
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "head_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-type",
                    "joint_trajectory_controller/JointTrajectoryController",
                    "--param-file",
                    str(Path(get_package_share_directory(moveit_package)) / "config" / "ros2_controllers.yaml"),
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/medipick/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                ],
                output="screen",
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
