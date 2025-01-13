from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


ARGS = [
    DeclareLaunchArgument('headless', default_value='false', 
            description='Whether to launch gazebo in headless mode'),
    DeclareLaunchArgument('sensing', default_value='false', 
            description='Whether to launch sensing nodes'),  
    DeclareLaunchArgument('localization', default_value='false', 
            description='Whether to launch localization nodes'),
    
    
    SetLaunchConfiguration('use_sim_time', 'true'),
    SetLaunchConfiguration('dual_ekf_navsat_param_path', PathJoinSubstitution([
        FindPackageShare('mowbot_launch'),
        'config', 'localization', 'mowbot_robot_localization',
        'dual_ekf_navsat.param.yaml'
    ])),
    SetLaunchConfiguration('gnss_fuser_param_path', PathJoinSubstitution([
        FindPackageShare('mowbot_launch'),
        'config', 'sensing',
        'gnss_fuser.param.yaml'
    ])),
    SetLaunchConfiguration('imu_filter_madgwick_param_path', PathJoinSubstitution([
        FindPackageShare('mowbot_launch'),
        'config', 'sensing',
        'imu_filter_madgwick.param.yaml'
    ])),
        
        
]

def generate_launch_description():
    
    return LaunchDescription(ARGS + [
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mowbot_simulator_launch'), 
                    'launch', 
                    'gazebo_sim.launch.py'
                ])
            ),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
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
                    ('/cmd_vel_out', '/mowbot_base/cmd_vel_unstamped')
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
        ]),
        
        # Sensing
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mowbot_sensing_launch'),
                    'launch',
                    'sensing_gazebo.launch.py'
                ])
            ),
            launch_arguments={
            }.items(),
            condition=IfCondition(LaunchConfiguration('sensing'))
        ),

        # Localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mowbot_localization_launch'),
                    'launch',
                    'localization_gazebo.launch.py'
                ])
            ),
            launch_arguments={
            }.items(),
            condition=IfCondition(LaunchConfiguration('localization'))
        ),
    ])