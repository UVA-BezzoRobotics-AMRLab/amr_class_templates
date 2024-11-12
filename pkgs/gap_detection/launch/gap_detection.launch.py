from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='gap_detection',
      namespace='',
      executable='gap_detector',
      name='gap_detector',
      output='screen',
      parameters=[
        {'vehicle_width': .36},
        {'detection_threshold': .4},
        {'min_gap_size': .36}
      ])
  ])
