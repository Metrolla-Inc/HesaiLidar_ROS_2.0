from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # rviz_config=get_package_share_directory('hesai_ros_driver')+'/rviz/rviz2.rviz'
    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='', description='Override ROS frame ID for messages'),
        DeclareLaunchArgument('point_cloud_topic', default_value='', description='Override point cloud topic name'),
        DeclareLaunchArgument('packet_topic', default_value='', description='Override packet topic name'),
        DeclareLaunchArgument('imu_topic', default_value='', description='Override IMU topic name'),
        DeclareLaunchArgument('packet_loss_topic', default_value='', description='Override packet loss topic name'),
        Node(
            namespace='hesai_ros_driver',
            package='hesai_ros_driver',
            executable='hesai_ros_driver_node',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'point_cloud_topic': LaunchConfiguration('point_cloud_topic'),
                'packet_topic': LaunchConfiguration('packet_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'packet_loss_topic': LaunchConfiguration('packet_loss_topic'),
            }],
        ),
        # Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config])
    ])
