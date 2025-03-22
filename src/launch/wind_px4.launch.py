from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnShutdown
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

    # Процесс запуска keyboard_control.launch.py (будет запущен позже)
    keyboard_control_launch = ExecuteProcess(
        cmd=["ros2", "launch", "px4_keyboard_teleop_control", "keyboard_control.launch.py"],
        output="screen"
    )

    # Таймер (ждет 40 секунд перед запуском клавиатурного управления)
    delay_before_keyboard = TimerAction(
        period=40.0,  # Ждем 40 секунд после запуска PX4 и Gazebo
        actions=[keyboard_control_launch]
    )

    # Обработчик `Ctrl+C` (выключение всех процессов)
    shutdown_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                micro_xrce_agent,  # Завершаем MicroXRCEAgent
                px4_sitl,  # Завершаем PX4 SITL
                keyboard_control_launch,
            ]
        )
    )

    return LaunchDescription([
        micro_xrce_agent,
        px4_sitl,
        gz_spawn_turbine,
        gz_bridge,
        delay_before_keyboard,  # Ждет перед запуском клавиатуры
        shutdown_handler
    ])
