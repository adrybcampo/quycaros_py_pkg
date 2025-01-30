from launch import LaunchDescription 
from launch_ros.actions import Node 


def generate_launch_description(): 
    return LaunchDescription([ 
        Node( 
            package='quycaros_py_pkg', 
            executable='states_server', 
            name='states_server' 
        ), 
        Node( 
            package='quycaros_py_pkg', 
            executable='main_control', 
            name='main_control' 
        ), 
        Node( 
            package='quycaros_py_pkg', 
            executable='emotion_control', 
            name='emotion_control' 
        ), 
        Node( 
            package='quycaros_py_pkg', 
            executable='video_capture', 
            name='video_capture' 
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