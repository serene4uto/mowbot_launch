from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    
    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mowbot_simulator_launch'), 
                    'launch', 
                    'gazebo_sim.launch.py'
                ])
            ),
            launch_arguments={
                'world_path': PathJoinSubstitution([
                    FindPackageShare('mowbot_simulator_launch'),
                    'worlds',
                    'sonoma_raceway.world'
                ]),
                'robot_name': 'mowbot',
                'x': '0.0',
                'y': '0.0',
                'z': '2.0',
                'yaw': '0.0'
            }.items()
        ),
        
        GroupAction([
            # twist_mux
            Node(
                name='twist_mux',
                package='twist_mux',
                executable='twist_mux',
                output='screen',
                remappings={
                    ('/cmd_vel_out', '/mowbot_velocity_controller/cmd_vel_unstamped')
                },
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare('mowbot_launch'), 'config', 'twist_mux.yaml']
                    )
                ]
            ),

            # teleop
            Node(
                package='joy',
                executable='joy_node',
                output='screen',
                name='joy_node',
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare('mowbot_launch'), 'config', 'teleop.yaml']
                    )
                ]
            ),

            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                output='screen',
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare('mowbot_launch'), 'config', 'teleop.yaml']
                    )
                ],
                remappings=[
                    ('/cmd_vel', '/joy_cmd_vel')
                ]
            ),
        ])
    ])