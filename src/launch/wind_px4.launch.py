from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Пути к файлам
    model_sdf_path = os.path.join(
        get_package_share_directory('drone_sim_environment'),
        'models',
        'wind_turbine_1',
        'model.sdf'
    )
    gz_bridge_config_file = os.path.join(
        get_package_share_directory("drone_sim_environment"), 'config', 'ros_gz_bridge.yaml'
    )

    # Процесс MicroXRCEAgent
    micro_xrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888",
             "--buffer-size", "4096",
             "--heartbeat-period-ms", "200",
             "--retries", "10", ],
        output="screen"
    )

    # Процесс PX4 SITL
    px4_sitl = ExecuteProcess(
        cmd=["bash", "-c", "cd ~/drone_autopilot/PX4-Autopilot && make px4_sitl gz_x500_gimbal_baylands"],
        output="screen"
    )

    offboard_control = Node(
            package="px4_offboard",
            executable="velocity_control",
            name='velocity'
        )

    keyboard_teleop=Node(
        package='px4_offboard',
        # namespace='px4_offboard',
        executable='control',
        name='control',
        prefix='gnome-terminal --',
    )

    # Спавн модели в Gazebo
    gz_spawn_turbine = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-file", model_sdf_path, "-name", "wind_turbine_1", "-x", "10.0", "-y", "10.0"],
        output="screen"
    )

    # Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gz_bridge_config_file}],
        output='screen'
    )

    # Обработчик выхода (чтобы PX4 завершался при выходе)
    kill_px4_on_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=micro_xrce_agent,  # Когда агент завершится
            on_exit=[px4_sitl],  # Завершаем PX4
        )
    )

    # Обработчик `Ctrl+C` (выключение всех процессов)
    shutdown_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                micro_xrce_agent,  # Завершаем MicroXRCEAgent
                px4_sitl,  # Завершаем PX4 SITL
            ]
        )
    )

    return LaunchDescription([
        micro_xrce_agent,
        offboard_control,
        keyboard_teleop,
        px4_sitl,
        gz_spawn_turbine,
        gz_bridge,
        kill_px4_on_exit,
        shutdown_handler
    ])