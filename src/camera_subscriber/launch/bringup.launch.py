from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    camera_node = Node(
        package='usb_cam',            # ← zmień jeśli inna kamera
        executable='usb_cam_node_exe',
        name='camera',
        output='screen'
    )

    aruco_controller = Node(
        package='camera_subscriber',
        executable='camera_node',
        name='aruco_controller',
        output='screen'
    )

    return LaunchDescription([
        camera_node,
        aruco_controller
    ])
