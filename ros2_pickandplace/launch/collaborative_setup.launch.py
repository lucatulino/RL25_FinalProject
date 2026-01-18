import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, AppendEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros2_pickandplace = get_package_share_directory('ros2_pickandplace')
    pkg_iiwa_description = get_package_share_directory('iiwa_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    logic_params_path = os.path.join(pkg_ros2_pickandplace, 'config', 'params.yaml')
    controllers_file_path = os.path.join(pkg_ros2_pickandplace, 'config', 'dual_controllers.yaml')

    xacro_file = os.path.join(pkg_iiwa_description, 'urdf', 'collaborative_setup.urdf.xacro')
    world_file = os.path.join(pkg_iiwa_description, 'gazebo', 'worlds', 'collaborative.world')
    turtlebot_model_path = os.path.join(pkg_iiwa_description, 'gazebo', 'models', 'turtlebot_ignition.sdf')
    aruco_201_path = os.path.join(pkg_iiwa_description, 'gazebo', 'models', 'arucotag', 'model.sdf')
    aruco_101_path = os.path.join(pkg_iiwa_description, 'gazebo', 'models', 'arucotag_101', 'model.sdf')

    try:
        pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
        turtlebot3_share_path = os.path.dirname(pkg_turtlebot3_description)
        turtlebot_urdf_path = os.path.join(pkg_turtlebot3_description, 'urdf', 'turtlebot3_burger.urdf')
    except:
        turtlebot3_share_path = ''
        turtlebot_urdf_path = ''

    set_env_resources_ign = AppendEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=turtlebot3_share_path)
    set_env_resources_gz = AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=turtlebot3_share_path)

    gz_args_arg = DeclareLaunchArgument('gz_flags', default_value='-r -v 4')
    record_bag_arg = DeclareLaunchArgument('record_bag', default_value='false')
    bag_path_arg = DeclareLaunchArgument('bag_path', default_value='my_bag')

    robot_description_content = Command([FindExecutable(name="xacro"), " ", xacro_file])
    
    node_robot_state_publisher = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher", 
        output="screen", 
        parameters=[{"robot_description": robot_description_content, "use_sim_time": True}]
    )
    
    node_turtlebot_state_publisher = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher", 
        name="turtlebot_state_publisher", 
        output="screen", 
        parameters=[{"robot_description": Command([FindExecutable(name="xacro"), " ", turtlebot_urdf_path]), "use_sim_time": True}], 
        remappings=[('/robot_description', '/turtlebot_description'), ('/joint_states', '/turtlebot_joint_states')]
    )

    static_tf = Node(
        package='tf2_ros', 
        executable='static_transform_publisher', 
        arguments=['0', '1.2', '0', '0', '0', '0', 'world', 'base_footprint'],
        output='screen'
    )
    
    static_tf_camera = Node(
        package='tf2_ros', 
        executable='static_transform_publisher', 
        arguments=['0.11', '0', '0.11', '0', '1.57', '0', 'base_footprint', 'camera_down_link'],
        output='screen'
    )

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')), 
        launch_arguments={'gz_args': [LaunchConfiguration('gz_flags'), ' ', world_file]}.items()
    )

    spawn_kuka = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=['-topic', 'robot_description', '-name', 'collaborative_cell', '-z', '0.0', '-allow_renaming', 'true'],
        output='screen'
    )
    
    spawn_turtlebot = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=['-file', turtlebot_model_path, '-name', 'turtlebot3_burger', '-x', '0.75', '-y', '0.7', '-z', '0.01', '-Y', '-1.57'],
        output='screen'
    )
    
    spawn_aruco_101_turtle = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=['-name', 'arucotag_101_turtle', '-file', aruco_101_path, '-x', '0.75', '-y', '0.6', '-z', '0.001'],
        output='screen'
    )
    
    spawn_aruco_101_right = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=['-name', 'arucotag_101_right', '-file', aruco_101_path, '-x', '0.75', '-y', '-0.63', '-z', '0.001'],
        output='screen'
    )
    
    spawn_aruco_201_air = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=['-name', 'arucotag_201_air', '-file', aruco_201_path, '-x', '0.75', '-y', '0.73', '-z', '0.25'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge', 
        arguments=[
            '/left_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image', 
            '/left_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo', 
            '/right_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image', 
            '/right_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo', 
            '/model/turtlebot3_burger/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry', 
            '/model/turtlebot3_burger/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist', 
            '/model/turtlebot3_burger/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model', 
            '/model/turtlebot3_burger/camera_down@sensor_msgs/msg/Image[gz.msgs.Image', 
            '/model/turtlebot3_burger/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo', 
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', 
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ], 
        remappings=[
            ('/model/turtlebot3_burger/odometry', '/odom'), 
            ('/model/turtlebot3_burger/cmd_vel', '/cmd_vel'), 
            ('/model/turtlebot3_burger/joint_states', '/turtlebot_joint_states'), 
            ('/model/turtlebot3_burger/camera_down', '/turtlebot_camera/image_raw'), 
            ('/model/turtlebot3_burger/camera_info', '/turtlebot_camera/camera_info')
        ],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["joint_state_broadcaster", "--param-file", controllers_file_path],
        output='screen'
    )
    
    controller_spawners = [
        Node(
            package="controller_manager", 
            executable="spawner", 
            arguments=[c, "--param-file", controllers_file_path],
            output='screen'
        ) 
        for c in ["left_velocity_controller", "right_velocity_controller", "left_gripper_controller", "right_gripper_controller"]
    ]

    aruco_node_left = Node(
        package='aruco_ros', 
        executable='single', 
        name='aruco_single_left', 
        parameters=[{
            'marker_id': 201, 
            'marker_size': 0.05, 
            'reference_frame': 'left_link_0', 
            'camera_frame': 'left_camera_link_optical', 
            'marker_frame': 'detected_marker_left', 
            'use_sim_time': True
        }], 
        remappings=[
            ('/image', '/left_camera/image_raw'), 
            ('/camera_info', '/left_camera/camera_info'), 
            ('/aruco_single/pose', '/aruco_single_left/pose')
        ],
        output='screen'
    )
    
    aruco_node_right = Node(
        package='aruco_ros', 
        executable='single', 
        name='aruco_single_right', 
        parameters=[{
            'marker_id': 201, 
            'marker_size': 0.05, 
            'reference_frame': 'right_link_0', 
            'camera_frame': 'right_camera_link_optical', 
            'marker_frame': 'detected_marker_right', 
            'use_sim_time': True
        }], 
        remappings=[
            ('/image', '/right_camera/image_raw'), 
            ('/camera_info', '/right_camera/camera_info'), 
            ('/aruco_single/pose', '/aruco_single_right/pose')
        ],
        output='screen'
    )
    
    aruco_node_turtle = Node(
        package='aruco_ros', 
        executable='single', 
        name='aruco_single_turtle', 
        parameters=[{
            'marker_id': 101, 
            'marker_size': 0.025, 
            'reference_frame': 'base_footprint', 
            'camera_frame': 'camera_down_link', 
            'marker_frame': 'detected_marker_101', 
            'image_is_rectified': True, 
            'use_sim_time': True
        }], 
        remappings=[
            ('/image', '/turtlebot_camera/image_raw'), 
            ('/camera_info', '/turtlebot_camera/camera_info'), 
            ('/aruco_single/result', '/aruco_single_turtle/result'), 
            ('/aruco_single/pose', '/aruco_single_turtle/pose')
        ],
        output='screen'
    )

    turtlebot_node = Node(
        package='ros2_pickandplace', 
        executable='turtlebot_patrol_node', 
        name='turtlebot_patrol', 
        parameters=[{'use_sim_time': True}, logic_params_path],
        output='screen'
    )
    
    left_logic_node = Node(
        package='ros2_pickandplace', 
        executable='iiwa_collaborative_node', 
        name='logic_node_left', 
        parameters=[
            {'robot_prefix': 'left_', 'base_link': 'left_link_0', 'tool_link': 'left_tool0'}, 
            logic_params_path
        ], 
        remappings=[('/aruco_single/pose', '/aruco_single_left/pose')],
        output='screen'
    )
    
    right_logic_node = Node(
        package='ros2_pickandplace', 
        executable='iiwa_collaborative_node', 
        name='logic_node_right', 
        parameters=[
            {'robot_prefix': 'right_', 'base_link': 'right_link_0', 'tool_link': 'right_tool0'}, 
            logic_params_path
        ], 
        remappings=[('/aruco_single/pose', '/aruco_single_right/pose')],
        output='screen'
    )

    return LaunchDescription([
        gz_args_arg, 
        record_bag_arg, 
        bag_path_arg, 
        set_env_resources_ign, 
        set_env_resources_gz, 
        node_robot_state_publisher, 
        node_turtlebot_state_publisher, 
        static_tf, 
        static_tf_camera, 
        ignition, 
        bridge, 
        TimerAction(period=5.0, actions=[spawn_kuka]), 
        TimerAction(period=7.0, actions=[spawn_turtlebot]), 
        TimerAction(period=9.0, actions=[spawn_aruco_101_turtle]), 
        TimerAction(period=10.0, actions=[spawn_aruco_101_right]), 
        TimerAction(period=11.0, actions=[spawn_aruco_201_air]), 
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_kuka, 
                on_exit=[TimerAction(period=2.0, actions=[joint_state_broadcaster])]
            )
        ), 
        TimerAction(period=12.0, actions=controller_spawners), 
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_spawners[0], 
                on_exit=[aruco_node_left, aruco_node_right, aruco_node_turtle, left_logic_node, right_logic_node, turtlebot_node]
            )
        ), 
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_path'), 
                '/tf', '/tf_static', '/clock', 
                '/aruco_single_turtle/result/compressed', 
                '/aruco_single_left/result/compressed', 
                '/aruco_single_right/result/compressed'
            ], 
            condition=IfCondition(LaunchConfiguration('record_bag')),
            output='screen'
        )
    ])
