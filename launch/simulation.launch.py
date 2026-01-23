from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['gz', 'sim', '-r', 'visualize_lidar.sdf'], output='screen'),
        
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/lidar@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
             remappings=[('/lidar', '/lidar/points')], output='screen'),
             
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0','0','0','0','0','0','map','vehicle_blue/chassis/gpu_lidar']),
             
        Node(package='lidar_clustering_py', executable='cluster_node', output='screen'),
        
        Node(package='rviz2', executable='rviz2', output='screen')
    ])
