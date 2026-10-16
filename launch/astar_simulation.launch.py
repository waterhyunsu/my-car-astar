"""Start Gazebo, localization, A-star navigation and one RViz window."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('astar_navigation')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    nav2 = get_package_share_directory('nav2_bringup')
    model_paths = ['/usr/share/gazebo-11/models', os.path.join(tb3, 'models')]
    if os.environ.get('GAZEBO_MODEL_PATH'):
        model_paths.append(os.environ['GAZEBO_MODEL_PATH'])
    resources = ['/usr/share/gazebo-11']
    if os.environ.get('GAZEBO_RESOURCE_PATH'):
        resources.append(os.environ['GAZEBO_RESOURCE_PATH'])

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(share, 'maps', 'astar_map.yaml')),
        DeclareLaunchArgument('ros_domain_id', default_value='0'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', ':'.join(model_paths)),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', ':'.join(resources)),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3, 'launch', 'turtlebot3_world.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'localization_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': os.path.join(nav2, 'params', 'nav2_params.yaml'),
                'use_composition': 'False',
            }.items(),
        ),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'astar_navigation.rviz')],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='astar_navigation', executable='astar_node', name='astar_navigation',
             parameters=[{'use_sim_time': True}], output='screen'),
    ])
