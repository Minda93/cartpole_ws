"""
  Launch
    rclcpp_components package
      component_container
        cartpole_interface package
          interface::CartpoleInterface plugin
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
  container = ComposableNodeContainer(
    name='ComponentManager',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
      ComposableNode(
        package='cartpole_interface',
        plugin='interface::CartpoleInterface',
        name='cartpole_interface')
    ],
    output='screen',
  )

  return LaunchDescription([container])