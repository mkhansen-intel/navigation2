import os
import sys
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch.actions
import launch_ros.actions

def generate_launch_description():
    world = launch.substitutions.LaunchConfiguration('world')
    model_path = launch.substitutions.LaunchConfiguration('model_path',
								default=EnvironmentVariable(name='GAZEBO_MODEL_PATH'))
    return LaunchDescription([
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', world],
					env={'GAZEBO_MODEL_PATH':model_path},
        				output='screen'),
        launch_ros.actions.Node(package='rviz2', node_executable='rviz2')
	])
