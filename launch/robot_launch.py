from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description(): 
    return LaunchDescription([ 

        DeclareLaunchArgument('Host', default_value='192.168.211.125', description='IP adress for socket connection'),

        Node( 
            package='quycaros_py_pkg', 
            executable='states_server', 
            name='states_server' 
        ), 
        Node( 
            package='quycaros_py_pkg', 
            executable='main_control', 
            name='main_control', 
            parameters=[
                {'Host': LaunchConfiguration('Host')}
            ]
        ), 
        Node( 
            package='quycaros_py_pkg', 
            executable='emotion_control', 
            name='emotion_control' 
        ), 
        Node( 
            package='quycaros_py_pkg', 
            executable='navigation', 
            name='navigation' 
        ), 
        Node( 
            package='v4l2_camera', 
            executable='v4l2_camera_node', 
            name='v4l2_camera_node' 
        ),
    ]) 