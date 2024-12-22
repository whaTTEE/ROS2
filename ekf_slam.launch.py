from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # EKF 노드 실행
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['ekf.yaml']
        ),
        # SLAM 노드 실행 (slam_toolbox 사용 예제)
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=['slam_toolbox_config.yaml']
        ),
    ])
