import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
  joy_config = LaunchConfiguration('joy_config')
  joy_dev = LaunchConfiguration('joy_dev')
  config_filepath = LaunchConfiguration('config_filepath')

  return LaunchDescription([
    DeclareLaunchArgument('joy_config', default_value='xbox'),
    DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
    DeclareLaunchArgument('ns', default_value=TextSubstitution(text="obsv_drone")),
    DeclareLaunchArgument(
      'config_filepath', default_value=[
        TextSubstitution(text=os.path.join(
          get_package_share_directory('mbzirc_joystick101'), 'config', '')),
          TextSubstitution(text='params.yaml')]),
    Node(
      package='joy', executable='joy_node', 
      namespace=LaunchConfiguration('ns'), 
      name='joy_node', 
      parameters=[{
        'dev': joy_dev,
        'deadzone': 0.0,
        'autorepeat_rate': 30.0,
      }]
    ),
    Node(
      package='mbzirc_joystick101', executable='mbzirc_joystick101_node', 
      namespace=LaunchConfiguration('ns'),
      name='mbzirc_joystick101_node',
      output='screen',
      emulate_tty=True,
      parameters=[config_filepath]
    )
  ])