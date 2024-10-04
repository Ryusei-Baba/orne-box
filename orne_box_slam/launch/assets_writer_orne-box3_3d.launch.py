import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 固定パスやファイル名をLaunchArgumentにすることで柔軟性を持たせる
    bag_dir = '/home/mochizuki12/bags/'
    default_bag_filename = '2024-03-20-20-58-06.bag'

    # 必要なLaunchConfigurationsを定義
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    cartographer_prefix = get_package_share_directory('orne_box_slam')
    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(cartographer_prefix, 'config', 'cartographer')
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename', 
        default='assets_writer_orne-box_3d.lua'
    )
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    
    rviz_config_dir = os.path.join(cartographer_prefix, 'config', 'rviz', 'cartographer.rviz')
    launch_include_file_dir = os.path.join(get_package_share_directory('orne_box_bringup'), 'launch/include')

    return LaunchDescription([
        # LaunchArgumentsの定義
        DeclareLaunchArgument(
            'bag_dir',
            default_value=bag_dir,
            description='Directory containing bag files'
        ),
        DeclareLaunchArgument(
            'bag_filename',
            default_value=default_bag_filename,
            description='Bag file name'
        ),
        DeclareLaunchArgument(
            'pose_graph_filename',
            default_value=PathJoinSubstitution([LaunchConfiguration('bag_dir'), LaunchConfiguration('bag_filename'), '.pbstream']),
            description='The pose graph file to process'
        ),

        # description.launch.pyのインクルード
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_include_file_dir, 'description.launch.py')
            ),
        ),     

        # Nodeの定義
        Node(
            package='cartographer_ros',
            executable='cartographer_assets_writer',
            name='cartographer_assets_writer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename,
                '-bag_filenames', PathJoinSubstitution([LaunchConfiguration('bag_dir'), LaunchConfiguration('bag_filename')]),
                '-pose_graph_filename', LaunchConfiguration('pose_graph_filename')
            ]
        )
    ])
