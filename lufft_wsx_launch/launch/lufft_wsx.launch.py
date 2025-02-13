from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    param_path = Path(get_package_share_directory("lufft_wsx_launch"), "param", "lufft_wsx.yaml")

    lufft_ws_node = Node(
        package="lufft_wsx",
        executable="wsxxx",
        name="lufft_ws",
        parameters=[param_path],
        # arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    ld.add_action(lufft_ws_node)

    return ld
