from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package paths
    pkg_share = get_package_share_directory('turtlebot3_gazebo')
    track_world_path = os.path.join(
        '/home/jamshaid-iqbal/Lab7-8-part2/robot_world',
        'track_world.world'
    )
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock time'
        ),
        
        # Start Gazebo with the track world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', track_world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            shell=False
        ),
        
        # Spawn TurtleBot3 Waffle Pi
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3_waffle_pi',
                '-file', os.path.join(pkg_share, 'models', 'turtlebot3_waffle_pi', 'model.sdf'),
                '-x', '0.0', '-y', '0.0', '-z', '0.0',
                '-Y', '0.0'
            ],
            output='screen'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ''
            }],
            remappings=[
                ('/joint_states', '/turtlebot3_waffle_pi/joint_states')
            ]
        ),
        
        # Launch the line follower node
        Node(
            package='turtlebot_vision',
            executable='follower',
            name='line_follower',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_topic': '/camera/image_raw',
                'cmd_vel_topic': '/cmd_vel',
                'use_yellow_mask': False,          # IMPORTANT: Switch to black line tracking
                'gray_threshold': 80,              # Threshold for the black track
                'publish_debug_image': True,       # Publishes the visual feed
                'linear_x': 0.15,                  # Forward speed
                'angular_gain': 2.0                # Turning sharpness
            }]
            # Removed the bad /cmd_vel remapping here so it publishes to the correct topic
        )
    ])