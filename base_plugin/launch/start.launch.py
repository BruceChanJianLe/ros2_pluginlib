from launch import LaunchDescription
from launch_ros.actions import Node
# Launch Args
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# Load Params Yaml
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  plugin_loader_node = Node(
    package='base_plugin',
    executable='plugin_loader_node',
    name='plugin_loader_node',
    parameters=[os.path.join(get_package_share_directory('base_plugin'), 'config', 'params.yaml')],
  )

  # Define launch description
  ld = LaunchDescription()
  ld.add_action(plugin_loader_node)

  # Return launch description
  return ld