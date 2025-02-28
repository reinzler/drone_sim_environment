import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Путь к миру windy.sdf
    world_file = os.path.join(
        get_package_share_directory('drone_sim_environment'),  # Замените на имя вашего пакета
        'worlds',
        'windy.sdf'
    )

    # Аргументы для GUI и паузы при старте
    gz_gui_arg = DeclareLaunchArgument(
        'gz_gui', default_value='true', description='Enable Gazebo GUI'
    )
    gz_paused_arg = DeclareLaunchArgument(
        'gz_paused', default_value='false', description='Start Gazebo in paused state'
    )

    # Команда для запуска Gazebo Harmonic
    gz_sim_process = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            '-r',  # Запуск в режиме воспроизведения
            '-s' if LaunchConfiguration('gz_gui') == 'false' else '',  # Отключение GUI
            world_file
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_gui_arg,
        gz_paused_arg,
        gz_sim_process
    ])