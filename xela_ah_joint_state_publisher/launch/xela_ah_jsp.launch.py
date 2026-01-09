from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 기본 YAML 경로를 패키지 내 샘플로 지정(원하면 공백으로 바꿔도 됨)
    default_cfg = PathJoinSubstitution([
        FindPackageShare('xela_ah_joint_state_publisher'),
        'config',
        'xela_ah_jsp_config.yaml'
    ])

    return LaunchDescription([
        # Robot description 소스 지정
        DeclareLaunchArgument('robot_description_from', default_value='topic'),  # 'topic' | 'param' | 'file'
        DeclareLaunchArgument('robot_description_topic', default_value='/robot_description'),
        DeclareLaunchArgument('robot_description_param', default_value='robot_description'),
        DeclareLaunchArgument('urdf_file', default_value=''),  # path to .urdf or .xacro

        # ✅ 배열/맵 파라미터는 YAML로만 전달
        DeclareLaunchArgument('config_yaml', default_value=default_cfg),

        # 스칼라(문자열/숫자/불리언)만 여기서 전달
        DeclareLaunchArgument('preserve_input_order', default_value='true'),
        DeclareLaunchArgument('publish_rate', default_value='30.0'),
        DeclareLaunchArgument('output_topic', default_value='/joint_states'),

        Node(
            package='xela_ah_joint_state_publisher',
            executable='xela_ah_joint_state_publisher_node',
            name='xela_ah_joint_state_publisher',
            output='screen',
            parameters=[
                # 스칼라 파라미터만 dict로 전달
                {
                    'robot_description_from':   LaunchConfiguration('robot_description_from'),
                    'robot_description_topic':  LaunchConfiguration('robot_description_topic'),
                    'robot_description_param':  LaunchConfiguration('robot_description_param'),
                    'urdf_file':                LaunchConfiguration('urdf_file'),
                    'preserve_input_order':     LaunchConfiguration('preserve_input_order'),
                    'publish_rate':             LaunchConfiguration('publish_rate'),
                    'output_topic':             LaunchConfiguration('output_topic'),
                    'config_yaml':              LaunchConfiguration('config_yaml')
                },
            ]
        )
    ])
