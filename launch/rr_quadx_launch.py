from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'camera': "imx708",          # Camera index or name
                'role': 'viewfinder',        # Stream role: raw, still, video, viewfinder
                'format': 'SBGGR10_1X10',    # Pixel format (auto by default)
                'width': 640,                # Image width
                'height': 480,               # Image height
                'frame_rate': 30,            # Frame rate (fps)
                'camera_info_url': '/home/aaron/rr_quadx_ws/etc/imx708.yaml',       # Path/URL to calibration YAML
                'orientation': 180             # Camera orientation in degrees (0, 90, 180, 270)
            }]
        )
    ])