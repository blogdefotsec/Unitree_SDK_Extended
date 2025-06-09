import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    fishbot_navigation2_dir = get_package_share_directory('task')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(os.path.abspath('.'), 'src','task','maps','map','default.yaml'))
    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(fishbot_navigation2_dir,'config','robot.yaml'))

    rviz_config_dir = os.path.join(os.path.abspath('.'), 'src', 'navigation2', 'nav2_bringup', 'bringup' ,'rviz', 'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value=use_sim_time,description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('map',default_value=map_yaml_path,description='Full path to map file to load'),
        DeclareLaunchArgument('params_file',default_value=nav2_param_path,description='Full path to param file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        
        Node(
            package="dog_control",
            executable="dog_control_B1",
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('hesai_lidar'), 'launch'),
                                           '/hesai_lidar_launch.py'])
        ),
        
        #Node(
        #    package='rslidar_sdk',
        #    executable='rslidar_sdk_node',
        #    name='rslidar_sdk_node',
        #    output='screen'
        #),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #         get_package_share_directory('lio_sam_ros2'), 'launch'),
        #         '/relio.launch.py'])
        # ),

        TimerAction(period=2.0,actions= [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('lio_sam_ros2'), 'launch'),
                    '/relio.launch.py'])
            )
        ]),
        
        #Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2',
        #    arguments=['-d', rviz_config_dir],
        #    parameters=[{'use_sim_time': use_sim_time}],
        #    output='screen'
        #),

    ])