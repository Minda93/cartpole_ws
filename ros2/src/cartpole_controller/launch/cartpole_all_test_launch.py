"""
  Launch
    rclcpp_components package
      component_container
        cartpole_interface package
          interface::CartpoleInterface plugin
"""
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

  pkgsPath = FindPackageShare(["cartpole_controller"])

  container = ComposableNodeContainer(
    name='ComponentManager',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
      ComposableNode(
        package='cartpole_interface',
        plugin='interface::CartpoleInterface',
        name='cartpole_interface'),
      ComposableNode(
        package='cartpole_controller',
        plugin='cartpole::CartpoleController',
        parameters=[os.path.join(pkgsPath.find("cartpole_controller"),
          "config", "cartpole_controller_component.yaml")],
        name='cartpole_controller')
    ],
    output='screen',
  )

  return LaunchDescription([container])