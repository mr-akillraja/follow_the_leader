import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_drone_desc = get_package_share_directory('drone_sim_description')
    
    world_file = os.path.join(pkg_drone_desc, 'worlds', 'swarm_world.sdf')
    urdf_file = os.path.join(pkg_drone_desc, 'urdf', 'drone.urdf.xacro')
    
    # 1. Start Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )
    
    # 2. Spawn 3 Drones and their low-level C++ controllers
    spawn_actions = []
    
    drone_names = ['alpha', 'beta_1', 'beta_2']
    
    for i, drone_name in enumerate(drone_names):
        # Process xacro for each drone to ensure unique naming if needed
        doc = xacro.process_file(urdf_file, mappings={'robot_name': drone_name})
        robot_desc = doc.toxml()
        
        # Spawn node in Gazebo
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', drone_name,
                '-string', robot_desc,
                '-x', str(-2.0 + (i+1)*1.5),
                '-y', '-2.0',
                '-z', '0.5'
            ],
            output='screen',
        )
        
        color = '0.0 1.0 1.0 1' if drone_name == 'alpha' else '0.6 0.6 0.6 1'
        
        # Robot State Publisher
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'rsp_{drone_name}',
            namespace=drone_name,
            output='screen',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', urdf_file, ' robot_name:=', drone_name, ' base_color:=', f'"{color}"']),
                value_type=str
            )}]
        )
        
        # C++ Low-level Controller Node
        controller_node = Node(
            package='drone_low_level',
            executable='drone_controller_node',
            name=f'controller_{drone_name}',
            output='screen',
            parameters=[{'drone_name': drone_name, 'alpha': 0.5}]
        )
        
        spawn_actions.extend([spawn_node, rsp_node, controller_node])

    # 3. Start Python High-level PSO Node
    pso_node = Node(
        package='pso_swarm_controller',
        executable='pso_node',
        name='pso_leader_node',
        output='screen'
    )

    # 4. Bridge for Gazebo clock and cmd_vel topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/alpha/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/beta_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/beta_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/alpha/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/beta_1/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/beta_2/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel_keyboard@geometry_msgs/msg/Twist[gz.msgs.Twist'
        ],
        remappings=[
            ('/model/alpha/pose', '/tf'),
            ('/model/beta_1/pose', '/tf'),
            ('/model/beta_2/pose', '/tf')
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        pso_node,
    ] + spawn_actions)
