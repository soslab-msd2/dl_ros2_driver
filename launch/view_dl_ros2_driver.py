import os
import launch

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    dl_ros2_driver_dir = os.path.join(get_package_share_directory('dl_ros2_driver'), 'launch', 'dl_ros2_driver.py')
    dl_ros2_driver = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(dl_ros2_driver_dir)
    )

    rviz_config_file = os.path.join(get_package_share_directory('dl_ros2_driver'), 'rviz', 'config.rviz')
    rviz_node = Node(
        node_name = 'rviz2',
        package = 'rviz2',
        node_executable = 'rviz2',
        output = 'screen',
        arguments = ['-d', rviz_config_file],
    )

    ld = launch.LaunchDescription()
    ld.add_action( dl_ros2_driver )
    ld.add_action( rviz_node )
    
    return ld
