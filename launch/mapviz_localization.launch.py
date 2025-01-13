from launch import LaunchDescription    
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    mapviz_config = PathJoinSubstitution(
        [FindPackageShare('mowbot_launch'), 
         'mapviz', 
         'mapviz_localization.mvc']
    )
    
    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('mowbot_visualization_launch'), 
                     'launch',
                     'mapviz.launch.py']
                )
            ),
            launch_arguments={
                'mvc_config': mapviz_config,
                'fix_topic': '/gnss/filtered',
            }.items()
        ),
        
    ])