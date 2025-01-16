from launch import LaunchDescription 
from launch_ros.actions import Node 


def generate_launch_description(): 
    return LaunchDescription([ 
        Node( 
            package='quyca_ros_pkg', 
            executable='states_server', 
            name='states_server' 
        ), 
        Node( 
            package='quyca_ros_pkg', 
            executable='main_control', 
            name='main_control' 
        ), 
        Node( 
            package='quyca_ros_pkg', 
            executable='emotion_control', 
            name='emotion_control' 
        ), 
        Node( 
            package='quyca_ros_pkg', 
            executable='video_capture', 
            name='video_capture' 
        ), 
        Node( 
            package='quyca_ros_pkg', 
            executable='navigation', 
            name='navigation' 
        ), 
    ]) 