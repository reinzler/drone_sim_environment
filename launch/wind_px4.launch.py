from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import subprocess
import time
from launch_ros.actions import Node


def generate_launch_description():
    # Путь к SDF модели
    model_sdf_path = os.path.join(
        get_package_share_directory('drone_sim_environment'),  # Замените на имя вашего пакета
        'models',
        'wind_turbine_1',
        'model.sdf'
    )

    MicroXRCEAgent_command = [
        "MicroXRCEAgent udp4 -p 8888",
    ]

    agent_command = " ".join(MicroXRCEAgent_command)
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", agent_command + "; exec bash"])

    # Pause between each command
    time.sleep(1)

    # Ensure `SDF_PATH` is populated
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]
        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # Запуск PX4 SITL через команду make px4_sitl gz_x500
    px4_sitl_command = [
        "cd ~/drone_autopilot/PX4-Autopilot && make px4_sitl gz_x500_windy",
    ]

    px4_command = " ".join(px4_sitl_command)
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", px4_command + "; exec bash"])

    # Pause between each command
    time.sleep(5)

    # Спавн модели в Gazebo
    gz_spawn_turbine = Node(
        package= "ros_gz_sim",
        executable="create",
        arguments=["-file", model_sdf_path, "-name", "wind_turbine_1", "-x", "10.0", "-y", "10.0"],
        output="screen"
    )

    return LaunchDescription([
        gz_spawn_turbine
    ])

