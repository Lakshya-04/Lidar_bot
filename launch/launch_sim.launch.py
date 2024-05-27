import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnExecutionComplete
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dexter = get_package_share_directory('dexter')

    # Process the URDF xacro file
    xacro_file = os.path.join(pkg_dexter,'description','robot.urdf.xacro')
    urdf = xacro.process_file(xacro_file)
    
    # Include the robot_state_publisher to publish transforms
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True},{'robot_description': urdf.toxml()}],
        emulate_tty=True
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    world_file_path = os.path.join(pkg_dexter, 'worlds', 'empty_world.world') 
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_file_path}.items()
    )

    # Run the spawner node from the gazebo_ros package to spawn the robot in the simulation
    spawn_entity = Node(
                package='gazebo_ros', 
                executable='spawn_entity.py',
                output='screen',
                arguments=['-topic', 'robot_description',   # The the robot description is published by the rsp node on the /robot_description topic
                           '-entity', 'dexter'],        # The name of the entity to spawn (doesn't matter if you only have one robot)
    )


    # Launch everything!
    return LaunchDescription([
        DeclareLaunchArgument(
          'use_sim_time',
          default_value='true',
          description='Use simulation/Gazebo clock'
        ),
        rsp,
        gazebo,
        
        spawn_entity,
        #slam,
    ])