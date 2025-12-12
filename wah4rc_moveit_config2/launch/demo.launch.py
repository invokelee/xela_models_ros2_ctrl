from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch argument 선언
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'hand',
            default_value='left',
            description='Hand type: left or right'
        )
    )
    
    # Argument를 xacro에 전달하면서 MoveIt config 빌드
    moveit_config = (
        MoveItConfigsBuilder("allegro_hand", package_name="wah4rc_moveit_config2")
        .robot_description(
            file_path="config/allegro_hand.urdf.xacro",  # 파일 경로 명시
            mappings={"hand": LaunchConfiguration('hand')}
        )
        .to_moveit_configs()
    )
    
    # 기존 generate_demo_launch 사용
    demo_launch = generate_demo_launch(moveit_config)
    
    # Argument와 demo launch 결합
    return LaunchDescription(declared_arguments + demo_launch.entities)

# ---------------------------------
# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("allegro_hand", package_name="wah4rc_moveit_config2").to_moveit_configs()
#     return generate_demo_launch(moveit_config)
