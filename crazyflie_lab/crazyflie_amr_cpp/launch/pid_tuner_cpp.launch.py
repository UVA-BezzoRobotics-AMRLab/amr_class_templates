
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():


  crazyflie_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('crazyflie'), 'launch'),
        '/launch_amr_lab.py'])
  )

  fly_node = Node(
              package='crazyflie_amr_cpp',
              namespace='',
              executable='fly_node',
              name='fly_node')

  pid_tuner = Node(
              package='crazyflie_amr_cpp',
              namespace='',
              executable='pid_tuner_cpp',
              name='pid_tuner')

  return LaunchDescription([
    crazyflie_launch,
    fly_node,
    pid_tuner
  ])