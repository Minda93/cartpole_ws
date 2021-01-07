"""
  Launch
    rclcpp_components package
      component_container
        cartpole_controller package
          cartpole::CartpoleController plugin
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
        package='cartpole_controller',
        plugin='cartpole::CartpoleController',
        name='cartpole_controller')
    ],
    output='screen',
  )

  return LaunchDescription([container])