import os
import sys
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    world = launch.substitutions.LaunchConfiguration('world')
    return LaunchDescription([
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', world],
        					output='screen'),
        launch_ros.actions.Node(package='rviz2', node_executable='rviz2')
	])
