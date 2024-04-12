from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    robot_model_arg = DeclareLaunchArgument(name='robot_model', default_value='locobot_wx250s',
                                            description='Robot model name')
    use_sim_time_arg = DeclareLaunchArgument(name='use_sim_time', default_value='true',
                                             description='Use simulation (Gazebo) clock if true')

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
        ])]),
        launch_arguments={
            'verbose': 'true',
            'world': PathJoinSubstitution([
                FindPackageShare('interbotix_xslocobot_gazebo'), 'worlds', 'interbotix_worlds', 'empty.world'
            ])
        }.items()
    )

    # Include robot description launch (for RViz and MoveIt)
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('interbotix_xslocobot_descriptions'), 'launch',
            'xslocobot_description.launch.py'
        ])]),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_rviz': 'true',
        }.items()
    )

    # MoveIt launch
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('interbotix_xslocobot_moveit'), 'launch',
            'xslocobot_moveit.launch.py'
        ])]),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # Assemble the launch description
    ld = LaunchDescription([
        robot_model_arg,
        use_sim_time_arg,
        gazebo_launch,
        robot_description_launch,
        moveit_launch,
    ])

    return ld
