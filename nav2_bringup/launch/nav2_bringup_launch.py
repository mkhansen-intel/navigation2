import os
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    map_file = launch.substitutions.LaunchConfiguration('map')
    map_type = launch.substitutions.LaunchConfiguration('map_type')
    return LaunchDescription([
        launch.actions.TimerAction(
	    actions = [
		launch_ros.actions.Node( package='nav2_map_server', 
					node_executable='map_server', 
					output='screen', 
					arguments=[map_file, map_type])
                 ], period = 1.0),
        launch.actions.TimerAction(
            actions = [
                launch_ros.actions.Node( package='dwb_controller', 
					node_executable='dwb_controller', 
					output='screen', 
					remappings=[('/cmd_vel', 'tb3/cmd_vel')])
                ], period = 5.0),
        launch.actions.TimerAction(
            actions = [
                launch_ros.actions.Node( package='nav2_dijkstra_planner', node_executable='dijkstra_planner', output='screen')
                ], period = 10.0),
        launch.actions.TimerAction(
            actions = [
                launch_ros.actions.Node( package='nav2_simple_navigator', node_executable='simple_navigator', output='screen')
                ], period = 15.0),
        launch.actions.TimerAction(
            actions = [
                launch_ros.actions.Node( package='nav2_mission_executor', node_executable='mission_executor', output='screen')
                ], period = 20.0),
        launch.actions.TimerAction(
            actions = [
                launch_ros.actions.Node( package='nav2_costmap_world_model', node_executable='costmap_world_model', output='screen')
                ], period = 25.0),
        launch.actions.TimerAction(
            actions = [
                launch_ros.actions.Node( package='nav2_amcl', node_executable='amcl', output='screen', remappings=[('scan', 'tb3/scan')])
                ], 
            period = 30.0),       
        
        launch_ros.actions.Node( package='tf2_ros', node_executable='static_transform_publisher', arguments='0 0 0 0 0 0 odom base_footprint'.split()),
        launch_ros.actions.Node( package='tf2_ros', node_executable='static_transform_publisher', arguments='0 0 0 0 0 0 base_footprint base_scan'.split())
    ])
