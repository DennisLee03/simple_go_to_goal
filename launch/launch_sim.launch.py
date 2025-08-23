import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node



def generate_launch_description():

    package_name='simple_go_to_goal'

    # ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /path/to/robot.urdf.xacro sim_mode:=true)"
    rsp = IncludeLaunchDescription(
                os.path.join(get_package_share_directory(package_name),'launch','rsp.launch.py'), 
                launch_arguments=[('use_sim_time', 'true')]
    )

    # ros2 launch gazebo_ros gazebo.launch.py
    gazebo_classic = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )

    # ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_bot
    # TODO: change to Ignition
    spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-timeout', '60'],
                        output='screen')

    return LaunchDescription([
        rsp,
        gazebo_classic,
        spawn_entity,
    ])