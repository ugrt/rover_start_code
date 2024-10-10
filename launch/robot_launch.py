import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node
from webots_ros2_driver.utils import controller_url_prefix

def generate_launch_description():
    package_dir = get_package_share_directory('rover_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    my_robot_driver = WebotsController(
        #package='webots_ros2_driver',
        #executable='driver',
        output='screen',
        #additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'Rover'},
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    main_run = Node(
        package='rover_package',
        executable='main_run',
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        main_run,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])