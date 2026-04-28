from pathlib import Path
import os
import re
import shlex

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
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


def set_joint_initial_value(urdf: str, joint_name: str, value: float) -> str:
    pattern = (
        rf'(<joint name="{re.escape(joint_name)}">\s*'
        r'<command_interface name="position" />\s*'
        r'<state_interface name="position">\s*'
        r'<param name="initial_value">)([^<]+)(</param>)'
    )
    updated_urdf, count = re.subn(
        pattern,
        lambda match: f"{match.group(1)}{value:.12f}{match.group(3)}",
        urdf,
        count=1,
    )
    if count == 0:
        raise RuntimeError(f"Expected ros2_control initial_value block for joint '{joint_name}'")
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
        "15.0",
        "--service-call-timeout",
        "15.0",
        "--switch-timeout",
        "15.0",
    ]
    if controller_type:
        arguments.extend(["--controller-type", controller_type])
    if ros2_controllers_file is not None:
        arguments.extend(["--param-file", str(ros2_controllers_file)])
    return arguments


def build_spawner_process(
    controller_name: str,
    ros2_controllers_file: Path | None = None,
    controller_type: str | None = None,
) -> ExecuteProcess:
    arguments = [
        "/opt/ros/humble/lib/controller_manager/spawner",
        *build_spawner_arguments(
            controller_name,
            ros2_controllers_file=ros2_controllers_file,
            controller_type=controller_type,
        ),
        "--ros-args",
        "-p",
        "use_sim_time:=True",
    ]
    controller_active_check = (
        "timeout 5s ros2 control list_controllers --spin-time 1 --controller-manager /controller_manager 2>/dev/null "
        r"| sed -r 's/\x1B\[[0-9;]*[A-Za-z]//g' "
        r"| awk '{print $1, $NF}' "
        f"| grep -Eq '^{controller_name}[[:space:]]+active$'"
    )
    retry_command = (
        "set -euo pipefail; "
        "for _ in $(seq 1 45); do "
        f"if {controller_active_check}; then exit 0; fi; "
        f"if timeout 20s {shlex.join(arguments)}; then exit 0; fi; "
        "sleep 2; "
        "done; "
        "exit 1"
    )
    return ExecuteProcess(
        cmd=["bash", "-lc", retry_command],
        output="screen",
    )


def build_controller_bootstrap_process(
    ros2_controllers_file: Path,
    *,
    spawn_mobile_arm_controller: bool = True,
    spawn_head_controller: bool = True,
    spawn_tool_controller: bool = True,
    ready_publish_controller_name: str | None = None,
) -> ExecuteProcess:
    controller_specs = [
        ("joint_state_broadcaster", None, None),
        ("imu_sensor_broadcaster", None, None),
        ("base_controller", "joint_trajectory_controller/JointTrajectoryController", ros2_controllers_file),
    ]
    if spawn_mobile_arm_controller:
        controller_specs.append(
            ("mobile_arm_controller", "joint_trajectory_controller/JointTrajectoryController", ros2_controllers_file)
        )
    if spawn_head_controller:
        controller_specs.append(("head_controller", "joint_trajectory_controller/JointTrajectoryController", ros2_controllers_file))
    if spawn_tool_controller:
        controller_specs.append(("tool_controller", "joint_trajectory_controller/JointTrajectoryController", ros2_controllers_file))

    ready_publish_controller = ready_publish_controller_name or controller_specs[-1][0]

    lines = [
        "set -euo pipefail",
        "controller_state() {",
        "  local name=\"$1\"",
        "  timeout 2s ros2 control list_controllers --spin-time 0.5 --controller-manager /controller_manager 2>/dev/null "
        "| sed -r 's/\\x1B\\[[0-9;]*[A-Za-z]//g' "
        "| awk -v target=\"$name\" '$1 == target {print $NF; exit}'",
        "}",
        "controller_active_check() {",
        "  [[ \"$(controller_state \"$1\" || true)\" == \"active\" ]]",
        "}",
        "activate_existing_controller() {",
        "  local name=\"$1\"",
        "  timeout 5s ros2 control set_controller_state \"$name\" active --controller-manager /controller_manager >/dev/null 2>&1 "
        "|| timeout 5s ros2 control switch_controllers --activate \"$name\" --controller-manager /controller_manager "
        "--switch-timeout 8.0 --activate-asap >/dev/null 2>&1",
        "}",
        "controller_manager_ready() {",
        "  timeout 2s ros2 control list_controllers --spin-time 0.5 --controller-manager /controller_manager >/dev/null 2>&1",
        "}",
        "ensure_controller() {",
        "  local name=\"$1\"",
        "  local type=\"$2\"",
        "  local params=\"$3\"",
        "  local attempt=0",
        "  for _ in $(seq 1 90); do",
        "    attempt=$((attempt + 1))",
        "    if ! controller_manager_ready; then",
        "      if (( attempt % 5 == 0 )); then",
        "        echo \"waiting for controller_manager before spawning ${name}...\"",
        "      fi",
        "      sleep 0.5",
        "      continue",
        "    fi",
        "    if controller_active_check \"$name\"; then",
        "      echo \"controller already active: ${name}\"",
        "      return 0",
        "    fi",
        "    local state=\"\"",
        "    state=$(controller_state \"$name\" || true)",
        "    if [[ -n \"$state\" ]]; then",
        "      echo \"controller already loaded: ${name} (${state}), trying to activate\"",
        "      activate_existing_controller \"$name\" || true",
        "      sleep 0.5",
        "      if controller_active_check \"$name\"; then",
        "        echo \"controller active after recovery: ${name}\"",
        "        return 0",
        "      fi",
        "    fi",
        "    cmd=(/opt/ros/humble/lib/controller_manager/spawner \"$name\" --controller-manager /controller_manager "
        "--controller-manager-timeout 8.0 --service-call-timeout 8.0 --switch-timeout 8.0)",
        "    if [[ -n \"$type\" ]]; then",
        "      cmd+=(--controller-type \"$type\")",
        "    fi",
        "    if [[ -n \"$params\" ]]; then",
        "      cmd+=(--param-file \"$params\")",
        "    fi",
        "    cmd+=(--ros-args -p use_sim_time:=True)",
        "    echo \"spawning controller: ${name}\"",
        "    if timeout 12s \"${cmd[@]}\"; then",
        "      for _state_try in $(seq 1 5); do",
        "        if controller_active_check \"$name\"; then",
        "          echo \"controller active: ${name}\"",
        "          return 0",
        "        fi",
        "        sleep 0.5",
        "      done",
        "    fi",
        "    state=$(controller_state \"$name\" || true)",
        "    if [[ -n \"$state\" ]]; then",
        "      echo \"controller present after spawn attempt: ${name} (${state}), trying to activate\"",
        "      activate_existing_controller \"$name\" || true",
        "      for _state_try in $(seq 1 5); do",
        "        if controller_active_check \"$name\"; then",
        "          echo \"controller active after post-spawn recovery: ${name}\"",
        "          return 0",
        "        fi",
        "        sleep 0.5",
        "      done",
        "    fi",
        "    sleep 1",
        "  done",
        "  echo \"controller bootstrap failed: ${name}\" >&2",
        "  exit 1",
        "}",
    ]

    for controller_name, controller_type, params_file in controller_specs:
        lines.append(
            "ensure_controller "
            f"{shlex.quote(controller_name)} "
            f"{shlex.quote(controller_type or '')} "
            f"{shlex.quote(str(params_file) if params_file is not None else '')}"
        )
        if controller_name == ready_publish_controller:
            lines.append(
                "for _publish_try in $(seq 1 30); do "
                "timeout 5s ros2 topic pub --once /medipick/controllers_ready std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true; "
                "sleep 0.2; "
                "done"
            )
        lines.append("sleep 1")

    return ExecuteProcess(
        cmd=["bash", "-lc", "\n".join(lines)],
        output="screen",
    )


def build_gazebo_robot_description(
    description_package: str,
    ros2_controllers_file: Path,
    gz_ros2_control_file: Path,
    *,
    initial_base_x: float = 0.0,
    initial_base_y: float = 0.0,
    initial_base_theta: float = 0.0,
) -> str:
    description_share_dir = Path(get_package_share_directory(description_package))
    base_urdf = load_file(description_package, "urdf/simple3_moveit.urdf")

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

    gazebo_urdf = add_raise_velocity_command_interface(gazebo_urdf)
    gazebo_urdf = set_joint_initial_value(gazebo_urdf, "base_x", initial_base_x)
    gazebo_urdf = set_joint_initial_value(gazebo_urdf, "base_y", initial_base_y)
    gazebo_urdf = set_joint_initial_value(gazebo_urdf, "base_theta", initial_base_theta)

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
      <visualize>false</visualize>
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
      <hold_joints>false</hold_joints>
      <position_proportional_gain>0.8</position_proportional_gain>
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


def launch_setup(context, *args, **kwargs):
    description_package = "medipick_simple3_description"
    moveit_package = "medipick_moveit_config"
    ros2_controllers_file = (
        Path(get_package_share_directory(moveit_package)) / "config" / "ros2_controllers_gazebo_trial.yaml"
    )
    gz_ros2_control_file = (
        Path(get_package_share_directory(moveit_package)) / "config" / "gz_ros2_control.yaml"
    )
    moveit_controllers_file = "config/moveit_controllers_gazebo_trial.yaml"

    world_file = str(
        Path(get_package_share_directory(description_package)) / "worlds" / "medipick_test.world.sdf"
    )
    initial_base_x = float(LaunchConfiguration("initial_base_x").perform(context))
    initial_base_y = float(LaunchConfiguration("initial_base_y").perform(context))
    initial_base_theta = float(LaunchConfiguration("initial_base_theta").perform(context))
    spawn_mobile_arm_controller = LaunchConfiguration("spawn_mobile_arm_controller").perform(context).lower() == "true"
    spawn_head_controller = LaunchConfiguration("spawn_head_controller").perform(context).lower() == "true"
    spawn_tool_controller = LaunchConfiguration("spawn_tool_controller").perform(context).lower() == "true"
    controllers_ready_after = LaunchConfiguration("controllers_ready_after").perform(context).strip()
    world_file_launch = LaunchConfiguration("world_file")
    rviz_config_file = str(Path(get_package_share_directory(moveit_package)) / "rviz" / "moveit.rviz")
    gz_sim_launch = str(Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py")

    robot_description = {
        "robot_description": build_gazebo_robot_description(
            description_package,
            ros2_controllers_file,
            gz_ros2_control_file,
            initial_base_x=initial_base_x,
            initial_base_y=initial_base_y,
            initial_base_theta=initial_base_theta,
        ),
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
    moveit_controllers = load_yaml(moveit_package, moveit_controllers_file)

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

    return [
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
                        parameters=[
                            robot_description,
                            {
                                "use_sim_time": True,
                                "publish_frequency": 100.0,
                                "ignore_timestamp": True,
                            },
                        ],
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
                    build_controller_bootstrap_process(
                        ros2_controllers_file,
                        spawn_mobile_arm_controller=spawn_mobile_arm_controller,
                        spawn_head_controller=spawn_head_controller,
                        spawn_tool_controller=spawn_tool_controller,
                        ready_publish_controller_name=controllers_ready_after or None,
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


def generate_launch_description():
    description_package = "medipick_simple3_description"
    world_file = str(
        Path(get_package_share_directory(description_package)) / "worlds" / "medipick_test.world.sdf"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("gazebo_gui", default_value="false"),
            DeclareLaunchArgument("with_move_group", default_value="true"),
            DeclareLaunchArgument("bridge_pointcloud", default_value="false"),
            DeclareLaunchArgument("world_file", default_value=world_file),
            DeclareLaunchArgument("spawn_delay", default_value="2.0"),
            DeclareLaunchArgument("state_publish_delay", default_value="0.5"),
            DeclareLaunchArgument("robot_spawn_z", default_value="0.03"),
            DeclareLaunchArgument("controller_spawn_delay", default_value="8.0"),
            DeclareLaunchArgument("controller_spawn_stagger", default_value="2.0"),
            DeclareLaunchArgument("spawn_mobile_arm_controller", default_value="true"),
            DeclareLaunchArgument("spawn_head_controller", default_value="true"),
            DeclareLaunchArgument("spawn_tool_controller", default_value="true"),
            DeclareLaunchArgument("controllers_ready_after", default_value=""),
            DeclareLaunchArgument("initial_base_x", default_value="0.0"),
            DeclareLaunchArgument("initial_base_y", default_value="0.0"),
            DeclareLaunchArgument("initial_base_theta", default_value="0.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
