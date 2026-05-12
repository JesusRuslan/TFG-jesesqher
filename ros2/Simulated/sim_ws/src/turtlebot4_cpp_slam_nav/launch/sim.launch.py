# sim.launch.py
# Lanza el entorno completo de simulación con Nav2 y el planificador elegido.
#
# USO:
#   Default:
#       ros2 launch turtlebot4_cpp_slam_nav sim.launch.py planner:=dijkstra
#       ros2 launch turtlebot4_cpp_slam_nav sim.launch.py
#   
#   Others:
#       ros2 launch turtlebot4_cpp_slam_nav sim.launch.py planner:=astar
#
# PLANIFICADORES DISPONIBLES:
#   dijkstra    ->      nav2_dijkstra.yaml      :       (Dijkstra, baseline)
#   astar       ->      nav2_astar.yaml         :       (A*)
#   otros       ->      

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


# -- Argumentos del launch ---------------------------------------------------------------------------
ARGUMENTS = [
    DeclareLaunchArgument(
        'planner',
        default_value='dijkstra',
        choices=['dijkstra', 'astar'],
        description='Algoritmo de planificación de trayectorias'
    ),
    DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        choices=['warehouse', 'depot', 'maze'],
        description='Mundo simulado'
    ),
    DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Lanzar RViz'
    ),
    DeclareLaunchArgument(
        'smooth',
        default_value='true',
        choices=['true', 'false'],
        description='Activar suavizado de trayectorias'
    ),
]


def generate_launch_description():

    # -- Directorios de paquetes ---------------------------------------------------------------------------
    pkg_slam_nav = get_package_share_directory('turtlebot4_cpp_slam_nav')
    pkg_turtlebot4_ignition = get_package_share_directory('turtlebot4_ignition_bringup')
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')
    pkg_turtlebot4_viz = get_package_share_directory('turtlebot4_viz')

    # -- Configuraciones ---------------------------------------------------------------------------
    planner = LaunchConfiguration('planner')
    world = LaunchConfiguration('world')
    launch_rviz = LaunchConfiguration('launch_rviz')
    smooth = LaunchConfiguration('smooth')

    # -- Yaml del planificador elegido ---------------------------------------------------------------------------
    # Nav2 coge los parámetros de nuestro yaml y sobreescribe el yaml oficial al ejecutar
    # Construye la ruta al yaml según el argumento 'planner':
    #   dijkstra    ->      config/nav2_dijkstra.yaml
    #   astar       ->      config/nav2_astar.yaml
    
    smooth_str = PythonExpression([
        '"" if "', smooth, '" == "true" else "_nosmooth"'
    ])

    planner_yaml = PathJoinSubstitution([
        pkg_slam_nav, 'config', PythonExpression([
            '"nav2_" + "', planner, '" + "', smooth_str, '" + ".yaml"'
        ])
    ])


    # -- Gazebo + robot ---------------------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_turtlebot4_ignition, 'launch',
                         'turtlebot4_ignition.launch.py')
        ]),
        launch_arguments={
            'world': world,
            'nav2': 'false',            # lo lanzamos nosotros en este launch
            'localization': 'true',
            'rviz': 'false',            # lo lanzamos nosotros en este launch
        }.items()
    )

    # -- Nav2 con nuestro yaml ---------------------------------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_turtlebot4_navigation, 'launch', 'nav2.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': planner_yaml,     # nuestro yaml
        }.items()
    )

    # -- RViz ---------------------------------------------------------------------------
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(launch_rviz)
    )

    # -- LaunchDescription ---------------------------------------------------------------------------
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(nav2)
    ld.add_action(rviz_node)
    ld.add_action(LogInfo(msg=['Lanzado con planner: ', planner, '; rviz: ', launch_rviz, ' y smooth: ', smooth]))
    ld.add_action(LogInfo(msg=['Lanzado con yaml: ', planner_yaml]))
    return ld
