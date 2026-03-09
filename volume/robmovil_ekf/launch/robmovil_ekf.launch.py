from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

use_trajectory = False

splines = [
            [
                0.,   0.,  0., 0.,
                5.,   0., -4., -1.57,
                10., -4., -4., 3.14,
                15., -4.,  0., 1.57,
                20.,  0.,  0., 0.
            ],
            [
                0.,   0.,  0., 0.785,
                5.,   0., -4., 5.535,
                10., -4., -4., 3.925,
                15., -4.,  0., 2.335,
                20.,  0.,  0., 0.785
            ],
]


def generate_launch_description():
    debug = LaunchConfiguration('debug')
    detector = LaunchConfiguration('detector')

    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='true'),
        DeclareLaunchArgument('detector', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),

        Node(
            package='imu_laser',
            executable='landmark_detector',
            name='landmark_detector',
            output='screen',
            parameters=[{'publish_robot_frame': 'base_link_ekf'},
                        {"use_sim_time": True}], 
            condition=IfCondition(detector)
        ),

        Node(
            package='imu_laser',
            executable='landmark_detector',
            name='landmark_detector_gt',
            output='screen',
            parameters=[{'publish_robot_frame': 'base_link_gt'},
                        {"use_sim_time": True}], 
            remappings=[
                ('/landmarks_pointcloud', '/landmarks_pointcloud/groundtruth'),
                ('/landmarks', '/landmarks/groundtruth'),
            ],
            condition=IfCondition(detector)
        ),

        Node(
            package='modelo_omnidireccional',
            executable='omni_odometry_node',
            name='omni_odometry',
            output='screen',
            parameters=[{"use_sim_time": True}]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'front_laser'],
            parameters=[{"use_sim_time": True}]
        ),

        # EKF node
        Node(
            package='robmovil_ekf',
            executable='localizer',
            name='localizer',
            output='screen',
            parameters=[{'only_prediction': False},
                        {'min_landmark_size': 1},
                        {"use_sim_time": True},
                        {"ekf_frame": "odom"}],
        ),
        Node(
            package="lazo_cerrado",
            executable="trajectory_follower_cl",
            name="trajectory_follower_cl",
            output="screen",
            parameters=[
                {'use_sim_time': True},
                {'odometry_selection':"EKF"}, #EKF, ODOMETRY
                {"goal_selection": "PURSUIT_BASED"}, #FIXED_GOAL, TIME_BASED, PURSUIT_BASED
                {"fixed_goal_x": float(2.0)},
                {"fixed_goal_y": float(2.0)},
                {"fixed_goal_a": float(-0.785)}, # -1/2 * PI
            ],
        ),

        Node(
            package="lazo_cerrado",
            executable="trajectory_generator",
            name="trajectory_generator",
            output="screen",
            parameters=[
                {'use_sim_time': True},
                {'trajectory_type': 'spline'}, #sin or spline
                {"stepping": float(.1)},
                {"total_time": float(40.0)},
                {"amplitude": float(4)},
                {"cycles": float(2.0)},
                {'spline_waypoints': splines[0]}
            ],  
        ),
        # Note: each waypoint must have 4 values: time(sec), position_x(m), position_y(m), orientation(rad)
        Node(
            package="lazo_cerrado",
            executable="logger",
            name="logger",
            output="screen",
            parameters=[{'use_sim_time': True}]
        ),

    
    ])

"""
if use_trajectory:
Node(
    package="lazo_cerrado",
    executable="trajectory_follower_cl",
    name="trajectory_follower_cl",
    output="screen",
    parameters=[
        {'use_sim_time': True},
        {"goal_selection": "PURSUIT_BASED"}, #FIXED_GOAL, TIME_BASED, PURSUIT_BASED
        {"fixed_goal_x": float(2.0)},
        {"fixed_goal_y": float(2.0)},
        {"fixed_goal_a": float(-0.785)}, # -1/2 * PI
    ],
),

Node(
    package="lazo_cerrado",
    executable="trajectory_generator",
    name="trajectory_generator",
    output="screen",
    parameters=[
        {'use_sim_time': True},
        {'trajectory_type': 'spline'}, #sin or spline
        {"stepping": float(.1)},
        {"total_time": float(40.0)},
        {"amplitude": float(4)},
        {"cycles": float(2.0)},
        {'spline_waypoints': splines[0]}
    ],  
),
# Note: each waypoint must have 4 values: time(sec), position_x(m), position_y(m), orientation(rad)
Node(
    package="lazo_cerrado",
    executable="logger",
    name="logger",
    output="screen",
    parameters=[{'use_sim_time': True}]
),
"""
