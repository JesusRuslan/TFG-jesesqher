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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction, ExecuteProcess
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
    # Construye la ruta al yaml según los argumentos 'planner' y 'smooth':
    #   dijkstra    ->      config/nav2_dijkstra.yaml
    #               ->      config/nav2_dijkstra_nosmooth.yaml
    #   astar       ->      config/nav2_astar.yaml
    #               ->      config/nav2_astar_nosmooth.yaml
    
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

    # -- Initial pose estimate ---------------------------------------------------------------------------
    initial_pose = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once', '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    '{"header": {"frame_id": "map"}, '
                    '"pose": {"pose": '
                    '{"position": {"x": 0.0, "y": 0.0, "z": 0.0}, '
                    '"orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}}'
                ],
                output='screen'
            )
        ]
    )

    # -- Safety limit override ---------------------------------------------------------------------------
    # Dado que el robot no puede ir marcha atrás más de una determinada distancia por un límite de seguridad, lo vamos
    # a desactivar para poder tener mayor control sobre el movimiento del robot si fuera necesario
    safety_override = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/motion_control',
                 'safety_override',
                 'backup_only'],
            output='screen'
        )
        ]
    )

    # -- Undock ---------------------------------------------------------------------------
    undock = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal', '/undock',
                    'irobot_create_msgs/action/Undock',
                    '{}'
                ],
                output='screen'
            )
        ]
    )

    # -- 180º spin ---------------------------------------------------------------------------
    rotate = TimerAction(
        period=110.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal', '/spin',
                    'nav2_msgs/action/Spin',
                    '{"target_yaw": 3.14159}'
                ],
                output='screen'
            )
        ]
    )

    # -- Nodo de métricas ---------------------------------------------------------------------------
    dynamic_metrics_node = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='turtlebot4_cpp_slam_nav',
                executable='data_measure_dynamic',
                name='nav_metrics_dynamic_node',
                output='screen',
                parameters=[{
                    'results_dir': os.path.join(
                        os.path.expanduser('~'),
                        'TFG-jesesqher/ros2/Simulated/Results/'
                    ),
                    'planner_name': planner,
                    'smooth': PythonExpression(['"', smooth, '" == "true"'])
                }]
            )
        ]
    )

    # -- LaunchDescription ---------------------------------------------------------------------------
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(nav2)
    ld.add_action(rviz_node)
    ld.add_action(LogInfo(msg=['Lanzado con planner: ', planner, '; rviz: ', launch_rviz, ' y smooth: ', smooth]))
    ld.add_action(LogInfo(msg=['Lanzado con yaml: ', planner_yaml]))
    ld.add_action(initial_pose)
    ld.add_action(safety_override)
    ld.add_action(undock)
    #ld.add_action(rotate)
    ld.add_action(dynamic_metrics_node)
    return ld
