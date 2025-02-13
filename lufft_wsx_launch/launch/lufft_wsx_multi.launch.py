from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    param_path = Path(get_package_share_directory("lufft_wsx_launch"), "param", "lufft_wsx_multi.yaml")

    lufft_ws100_node = Node(
        package="lufft_wsx",
        executable="wsxxx",
        namespace="aws",
        name="lufft_ws100",
        remappings=[("wsxxx_measurements", "ws100_measurements")],
        parameters=[param_path],
    )

    lufft_ws501_node = Node(
        package="lufft_wsx",
        executable="wsxxx",
        namespace="aws",
        name="lufft_ws501",
        remappings=[("wsxxx_measurements", "ws501_measurements")],
        parameters=[param_path],
    )

    lufft_ws600_node = Node(
        package="lufft_wsx",
        executable="wsxxx",
        namespace="aws",
        name="lufft_ws600",
        remappings=[("wsxxx_measurements", "ws600_measurements")],
        parameters=[param_path],
    )

    ld.add_action(lufft_ws100_node)
    ld.add_action(lufft_ws501_node)
    # ld.add_action(lufft_ws600_node)

    return ld
