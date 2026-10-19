"""
Publishes the head camera of a Unitree robot (G1, H1) on /camera/image_raw, read through Unitree's videohub service,
which keeps running and owns the camera. By default the frames are downscaled to 960x540 and available as H.264
(transport "ffmpeg", ~1 Mbit/s), JPEG (transport "compressed"), and raw; each transport is encoded only while
subscribed. On the H1, whose camera is upside down, the images are rotated by 180 degrees. Run it on the onboard
computer, in a container on the host network (no access to the video devices is needed):

    ros2 launch unitree_videohub_bridge unitree_videohub_bridge_launch.py robot:=g1
    ros2 launch unitree_videohub_bridge unitree_videohub_bridge_launch.py robot:=h1
    ros2 launch unitree_videohub_bridge unitree_videohub_bridge_launch.py robot:=g1 downscale:=4      # 480x270
    ros2 launch unitree_videohub_bridge unitree_videohub_bridge_launch.py robot:=g1 transcode:=false  # JPEG as it comes

The parameters shared by the robots (rate, topic, frame_id, H.264 and JPEG quality) are in
config/unitree_videohub_bridge.yaml (another file can be given with config_file:=<file>), and the ones of each robot
(rotation) in config/<robot>.yaml. Check the frames with
    ros2 topic hz /camera/image_raw/ffmpeg
    ros2 topic bw /camera/image_raw/ffmpeg
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_directory = PathJoinSubstitution([FindPackageShare('unitree_videohub_bridge'), 'config'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            choices=['g1', 'h1'],
            description='The robot: selects config/<robot>.yaml (e.g. the rotation of the image).'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([config_directory, 'unitree_videohub_bridge.yaml']),
            description='YAML file with the parameters shared by the robots.'
        ),
        DeclareLaunchArgument(
            'domain_id',
            default_value='0',
            description='DDS domain of the Unitree network: 0 on the robot.'
        ),
        DeclareLaunchArgument(
            'network_interface',
            default_value='eth0',
            description='Interface on the Unitree network (192.168.123.x): "eth0" on the onboard computer.'
        ),
        DeclareLaunchArgument(
            'transcode',
            default_value='true',
            choices=['true', 'false'],
            description='true: downscale, rotate, and publish through image_transport (ffmpeg, compressed, raw). '
                        'false: the JPEG of videohub as it comes, on <topic>/compressed (not possible on the H1, '
                        'whose images must be rotated).'
        ),
        DeclareLaunchArgument(
            'downscale',
            default_value='2',
            choices=['1', '2', '4', '8'],
            description='The resolution (1920x1080) is divided by it when transcoding.'
        ),
        Node(
            package='unitree_videohub_bridge',
            executable='unitree_videohub_bridge_node',
            name='unitree_videohub_bridge',
            output='screen',
            # Later entries take precedence: the robot file overrides the shared one, and the launch arguments both.
            parameters=[LaunchConfiguration('config_file'),
                        PathJoinSubstitution([config_directory, [LaunchConfiguration('robot'), '.yaml']]),
                        {'domain_id': ParameterValue(LaunchConfiguration('domain_id'), value_type=int),
                         'network_interface': LaunchConfiguration('network_interface'),
                         'transcode': ParameterValue(LaunchConfiguration('transcode'), value_type=bool),
                         'downscale': ParameterValue(LaunchConfiguration('downscale'), value_type=int)}],
        ),
    ])
