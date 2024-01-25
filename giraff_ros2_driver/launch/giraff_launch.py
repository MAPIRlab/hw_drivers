from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.frontend.parse_substitution import parse_substitution

def generate_launch_description():

    use_odom = DeclareLaunchArgument('publish_odom', default_value="True")
    giraff = Node (
        package='giraff_ros2_driver',
        executable='giraff_node',
        name='giraff_node',
        output='screen',
        parameters=[
        {'giraff_avr_port':'/dev/ttyS1'},
        {'publish_odometry_over_tf':parse_substitution('$(var publish_odom)')},
        {'publish_other_tf':True},
        {'odom_topic':'odom'},
        {'freq': 100.0},
        {'verbose':False},
        
        {'base_frame_id':'giraff_base_link'},
        {'odom_frame_id':'giraff_odom'},
        {'stalk_frame_id':'giraff_neck'},
        {'head_frame_id':'giraff_head'},
        {'screen_frame_id':'giraff_screen'},
        {'base_footprint_frame_id':'giraff_base_footprint'},
        {'camera_frame_id':'giraff_camera'},
        {'laser_frame_id':'giraff_laser_frame'},
        
        {'controller_mode':0},
        {'max_linear_vel':0.7},
        {'max_angular_vel':0.7},
        {'linear_acceleration':0.2},
        {'angular_acceleration':0.4},
        {'virtual_gear_ratio':20.0},
        {'tilt_bias':0.6},
        {'battery_technology':'NIMH'},
        {'battery_design_capacity':4.4},
        {'battery_serial_number':'Giraff_battery'}
        ]
    )

    return LaunchDescription([
        use_odom,
        giraff
    ])