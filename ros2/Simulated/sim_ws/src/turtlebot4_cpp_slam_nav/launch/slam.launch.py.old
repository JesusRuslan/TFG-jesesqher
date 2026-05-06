import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from ament_index_python import get_package_share_directory


def generate_launch_description():

    slam = GroupAction([
        # Forzamos el namespace para que coincida con el del robot
        PushRosNamespace('turtlebot4'),

        # Vamos a usar slam_toolbox
        Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            # ROS2
            'use_sim_time': True,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': 'scan',
            'mode': 'mapping',

            # LIDAR
            'resolution': 0.05,             # 5 cm por celda
            'min_laser_range': 0.2,         # rango mínimo del LIDAR
            'max_laser_range': 12.0,        # rango máximo del LiDAR

            # Timing stuff
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0
        }],
        ),
    ])

    # Usamos también RViz para visualizar el mapa
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('turtlebot4_viz'),
            'rviz', 'nav2.rviz')],        # config de RViz del TurtleBot4
    )


    return LaunchDescription([
        slam,
        rviz_node,
    ])