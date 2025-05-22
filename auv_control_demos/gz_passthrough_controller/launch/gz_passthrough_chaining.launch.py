# Copyright 2024, Everardo Gonzalez
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description to run the chained controller demo.

    Returns:
        The example launch description.
    """

    # this file must be in the xacro/ folder
    xacro_filename = "main.urdf.xacro"
    # this file must be in the config/ folder
    config_filename = "gz_passthrough_chained.yaml"
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("auv_control_demos"),
                    "xacro",
                    xacro_filename,
                ]
            ),
            'yaml_file:=',
            config_filename,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # TODO place world file in the package directory instead
    world = os.path.join(get_package_share_directory('sisid_description'), 'worlds', 'sisid_world.sdf')
    gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
         )
    gz_sim_spawn = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'sisid_debug',
                                   '-z', '-0.1'],
                        output='screen')

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "integral_sliding_mode_controller",
            "--controller-manager",
            ["", "controller_manager"],
        ],
    )

    thruster_locations = ['left', 'right', 'vert']

    thruster_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                f"thruster_{location}_controller",
                "--controller-manager",
                ["", "controller_manager"],
            ],
        )
        for location in thruster_locations
    ]

    delay_thruster_spawners = []
    for i, thruster_spawner in enumerate(thruster_spawners):
        if not len(delay_thruster_spawners):
            delay_thruster_spawners.append(
                thruster_spawner,
            )
        else:
            delay_thruster_spawners.append(
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=thruster_spawners[i - 1],
                        on_exit=[thruster_spawner],
                    )
                )
            )

    tam_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "thruster_allocation_matrix_controller",
            "--controller-manager",
            ["", "controller_manager"],
        ],
    )
    delay_tam_controller_spawner_after_thruster_controller_spawners = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=thruster_spawners[-1],
                on_exit=[tam_controller_spawner],
            )
        )
    )

    delay_velocity_controller_spawner_after_tam_controller_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=tam_controller_spawner,
                on_exit=[velocity_controller_spawner],
            )
        )
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, "use_sim_time:=true"],
        ),
        # gz_sim,
        # gz_sim_spawn,
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("auv_control_demos"),
                        "config",
                        config_filename,
                    ]
                ),
            ],
            remappings=[
                ("/controller_manager/robot_description", "/robot_description"),
            ],
        ),
        *delay_thruster_spawners,
        delay_tam_controller_spawner_after_thruster_controller_spawners,
        delay_velocity_controller_spawner_after_tam_controller_spawner,
    ]

    return LaunchDescription(nodes)
