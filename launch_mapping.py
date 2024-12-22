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
            parameters=['/path/to/ekf_config.yaml']
        ),
        # SLAM Toolbox 실행
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=['/path/to/slam_config.yaml']
        ),
        # GPS 데이터 변환
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=['/path/to/ekf_config.yaml']
        )
    ])
