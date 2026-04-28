from pathlib import Path
import shutil
import zipfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _strip_cam_link_geometry(robot_description: str) -> str:
    visual_block = """    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://simple5_urdf/meshes/cam_link.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
"""
    collision_block = """    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://simple5_urdf/meshes/cam_link.STL" />
      </geometry>
    </collision>
"""
    return robot_description.replace(visual_block, "", 1).replace(collision_block, "", 1)


def _load_raw_simple5_robot_description() -> str:
    launch_path = Path(__file__).resolve()
    candidate_roots = [launch_path.parent.parent]
    candidate_roots.extend(launch_path.parents)
    candidate_roots.extend(Path.cwd().resolve().parents)
    candidate_roots.append(Path.cwd().resolve())

    asset_root = None
    for root in candidate_roots:
        candidate = root / "vendor" / "simple5_current"
        if (candidate / "urdf" / "simple5_urdf.urdf").exists():
            asset_root = candidate
            break

    for root in candidate_roots:
        if asset_root is not None:
            break
        candidate = root / "assets" / "simple5_urdf"
        if (candidate / "urdf" / "simple5_urdf.urdf").exists():
            asset_root = candidate
            break

    if asset_root is None:
        for root in candidate_roots:
            zip_path = root / "assets" / "simple5_urdf.zip"
            extract_root = root / "assets" / "simple5_urdf"
            if not zip_path.exists():
                continue
            if extract_root.exists():
                shutil.rmtree(extract_root)
            extract_root.mkdir(parents=True, exist_ok=True)
            with zipfile.ZipFile(zip_path) as archive:
                archive.extractall(extract_root)
            inner_dir = extract_root / "simple5_urdf"
            if inner_dir.exists():
                for child in inner_dir.iterdir():
                    shutil.move(str(child), str(extract_root / child.name))
                inner_dir.rmdir()
            if (extract_root / "urdf" / "simple5_urdf.urdf").exists():
                asset_root = extract_root
                break

    if asset_root is None:
        raise FileNotFoundError("未找到 assets/simple5_urdf.zip 或已解包的 simple5_urdf 资产目录")

    urdf_path = asset_root / "urdf" / "simple5_urdf.urdf"
    robot_description = urdf_path.read_text(encoding="utf-8")
    robot_description = _strip_cam_link_geometry(robot_description)
    robot_description = robot_description.replace(
        "package://simple5_urdf/",
        f"file://{asset_root.as_posix()}/",
    )
    return robot_description


def generate_launch_description():
    robot_description = _load_raw_simple5_robot_description()
    rviz_config = str(
        Path(get_package_share_directory("medipick_simple3_description"))
        / "rviz"
        / "simple5_asset_preview.rviz"
    )
    preview_joint_states_topic = "/simple5_asset_preview/joint_states"

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_config),
            Node(
                package="medipick_simple3_description",
                executable="simple5_asset_joint_state_publisher.py",
                name="simple5_asset_joint_state_publisher",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "joint_state_topic": preview_joint_states_topic,
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="simple5_asset_robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", preview_joint_states_topic)],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                output="screen",
            ),
        ]
    )
