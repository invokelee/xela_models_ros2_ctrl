# custom_demo_template.launch.py — Template for dual-model MoveIt demo
# - Full URDF (e.g., sensors/taxels ON) for TF/ros2_control
# - Light URDF (e.g., sensors OFF) for MoveIt planning
# HOW TO USE:
#   1) Set default robot_name/package_name/top_xacro to your moveit_config/xacro.
#   2) Adjust xacro mappings (taxel toggles, hw plugins, topics) as needed.

from pathlib import Path
import tempfile
import atexit
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Core args (edit defaults to match your package)
    robot_name_arg   = DeclareLaunchArgument("robot_name",   default_value="allegro_hand_left")
    package_name_arg = DeclareLaunchArgument("package_name", default_value="xalh4rc_moveit_config1")
    db_arg           = DeclareLaunchArgument("db",           default_value="false")
    use_rviz_arg     = DeclareLaunchArgument("use_rviz",     default_value="true")

    # Xacro control (dual)
    top_xacro_arg = DeclareLaunchArgument(
        "top_xacro",
        default_value="allegro_hand_left.urdf.xacro",  # TODO: change to your top xacro filename
        description="Filename only under <package>/config/."
    )
    taxels_full_arg   = DeclareLaunchArgument("allegro_taxels_full",   default_value="1")
    taxels_light_arg  = DeclareLaunchArgument("allegro_taxels_moveit", default_value="0")
    sensor_collision_arg = DeclareLaunchArgument("allegro_sensor_collision", default_value="0")
    ros2_control_hardware_type_arg = DeclareLaunchArgument("ros2_control_hardware_type", default_value="mock_components")

    # LCs
    robot_name       = LaunchConfiguration("robot_name")
    package_name     = LaunchConfiguration("package_name")
    use_rviz         = LaunchConfiguration("use_rviz")
    use_db           = LaunchConfiguration("db")
    top_xacro        = LaunchConfiguration("top_xacro")
    taxels_full      = LaunchConfiguration("allegro_taxels_full")
    taxels_light     = LaunchConfiguration("allegro_taxels_moveit")
    sensor_collision = LaunchConfiguration("allegro_sensor_collision")
    ros2_control_hardware_type = LaunchConfiguration("ros2_control_hardware_type")

    return LaunchDescription([
        robot_name_arg, package_name_arg, db_arg, use_rviz_arg,
        top_xacro_arg, taxels_full_arg, taxels_light_arg, sensor_collision_arg,
        ros2_control_hardware_type_arg,
        OpaqueFunction(
            function=_launch_setup,
            args=[robot_name, package_name, use_rviz, use_db, top_xacro,
                  taxels_full, taxels_light, sensor_collision, ros2_control_hardware_type,
                ]
        ),
    ])


def _launch_setup(context, robot_name, package_name, use_rviz, use_db, top_xacro,
                  taxels_full, taxels_light, sensor_collision, ros2_control_hardware_type,
                  ):
    # Resolve substitutions
    robot_name_v       = robot_name.perform(context)
    package_name_v     = package_name.perform(context)
    use_rviz_v         = use_rviz.perform(context).lower() in ("true", "1", "yes")
    use_db_v           = use_db.perform(context).lower() in ("true", "1", "yes")
    top_xacro_filename = top_xacro.perform(context)
    taxels_full_v      = taxels_full.perform(context)
    taxels_light_v     = taxels_light.perform(context)
    sensor_collision_v = sensor_collision.perform(context)
    ros2_control_hardware_type_v = ros2_control_hardware_type.perform(context)

    # Enforce filename-only for top_xacro
    if ("/" in top_xacro_filename) or ("\\" in top_xacro_filename):
        raise RuntimeError(f"top_xacro must be a filename under <pkg>/config/, got: {top_xacro_filename}")

    # Resolve package path and top xacro path
    pkg_path = Path(get_package_share_directory(package_name_v))
    top_xacro_path = pkg_path / "config" / top_xacro_filename
    if not top_xacro_path.exists():
        raise RuntimeError(f"Top xacro not found under <pkg>/config/: {top_xacro_path}")

    # Expand xacro once per model and reuse the rendered URDFs
    def _render_xacro_once(path: Path, mappings: dict) -> str:
        doc = xacro.process_file(str(path), mappings=mappings)
        xml = doc.toprettyxml(indent="  ")
        tmp = tempfile.NamedTemporaryFile(prefix="xela_urdf_", suffix=".urdf", delete=False)
        tmp.write(xml.encode())
        tmp.flush()
        tmp.close()
        atexit.register(lambda p=tmp.name: Path(p).exists() and Path(p).unlink())
        return tmp.name

    rendered_urdf_full = _render_xacro_once(
        top_xacro_path,
        {
            "taxels": taxels_full_v,
            "sensor_collision": sensor_collision_v,
            "hand": "left",
            "ros2_control_hardware_type": ros2_control_hardware_type_v,
        },
    )
    rendered_urdf_light = _render_xacro_once(
        top_xacro_path,
        {
            "taxels": taxels_light_v,
            "sensor_collision": sensor_collision_v,
            "hand": "left",
            "ros2_control_hardware_type": ros2_control_hardware_type_v,
        },
    )

    # Build MoveIt configs with the light model (matches SRDF)
    moveit_config = (
        MoveItConfigsBuilder(robot_name_v, package_name=package_name_v)
        .robot_description(file_path=str(rendered_urdf_light))
        .to_moveit_configs()
    )

    # Prepare parameter dictionaries
    robot_description_full = {"robot_description": Path(rendered_urdf_full).read_text()}

    actions = []
    post_can_actions = []

    if ros2_control_hardware_type_v == "physical_device":
        can_down = ExecuteProcess(cmd=["sudo", "ip", "link", "set", "can0", "down"])
        can_type = ExecuteProcess(cmd=["sudo", "ip", "link", "set", "can0", "type", "can", "bitrate", "1000000"])
        can_up = ExecuteProcess(cmd=["sudo", "ip", "link", "set", "can0", "up"])
        actions.append(can_down)
        actions.append(RegisterEventHandler(OnProcessExit(target_action=can_down, on_exit=[can_type])))
        actions.append(RegisterEventHandler(OnProcessExit(target_action=can_type, on_exit=[can_up])))

        def _after_can_up(event, context):
            if event.returncode != 0:
                return [EmitEvent(Shutdown(reason="can0 setup failed"))]
            return post_can_actions

        actions.append(RegisterEventHandler(OnProcessExit(target_action=can_up, on_exit=_after_can_up)))

    # (1) static_virtual_joint_tfs (optional)
    static_vj = pkg_path / "launch" / "static_virtual_joint_tfs.launch.py"
    if static_vj.exists():
        post_can_actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(static_vj))))

    # (1.5) sensor joint-state publisher (only when full model uses sensors)
    # sensor_jsp = Path(get_package_share_directory("xela_ah_joint_state_publisher")) / "launch" / "xela_ah_jsp.launch.py"
    # if sensor_jsp.exists() and str(taxels_full_v) == "1":
    #     post_can_actions.append(
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(str(sensor_jsp)),
    #             launch_arguments={
    #                 "robot_description_from": "topic",
    #                 "robot_description_topic": "/robot_description_full",
    #                 "output_topic": "/joint_states_full",
    #             }.items(),
    #         )
    #     )

    # (2) robot_state_publisher — light model for TF
    post_can_actions.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    ))

    # (2b) robot_state_publisher for full model publishing to /robot_description_full
    post_can_actions.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_full",
        output="screen",
        parameters=[robot_description_full],
        remappings=[
            ("joint_states", "joint_states_full"),
            ("robot_description", "robot_description_full"),
        ],
    ))

    # (2c) joint_state bridge: /joint_states -> /joint_states_full (sensor_data QoS)
    post_can_actions.append(Node(
        package="xalh4rc_moveit_config1",
        executable="joint_state_relay.py",
        name="joint_state_relay",
        output="screen",
    ))

    # (3) move_group — use light model (taxels=0)
    move_group_entities = generate_move_group_launch(moveit_config).entities
    post_can_actions.extend(move_group_entities)

    # (4) RViz (optional) — use MoveIt parameters (light)
    if use_rviz_v:
        rviz_candidate = pkg_path / "config" / "moveit.rviz"
        rviz_full_candidate = pkg_path / "config" / "robot_description_full.rviz"
        rviz_args = ["-d", str(rviz_candidate)] if rviz_candidate.exists() else []
        post_can_actions.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=rviz_args,
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
            ],
        ))
        rviz_full_args = ["-d", str(rviz_full_candidate)] if rviz_full_candidate.exists() else []
        post_can_actions.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_full_model",
            output="screen",
            arguments=rviz_full_args,
        ))

    # (5) warehouse_db (optional)
    if use_db_v:
        post_can_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg_path / "launch" / "warehouse_db.launch.py"))
        ))

    # (6) ros2_control node + spawners — unified (default) or split
    controllers_yaml = pkg_path / "config" / "ros2_controllers.yaml"
    if controllers_yaml.exists():
        post_can_actions.append(Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[robot_description_full, str(controllers_yaml)],
        ))
        post_can_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg_path / "launch" / "spawn_controllers.launch.py"))
        ))

    if ros2_control_hardware_type_v != "physical_device":
        actions.extend(post_can_actions)

    return actions
