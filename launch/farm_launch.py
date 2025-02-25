from simple_launch import SimpleLauncher, GazeboBridge
from ament_index_python.packages import get_package_share_directory
import os


sl = SimpleLauncher(use_sim_time = True)

sl.declare_arg('nx', default_value = 2, description = 'how many turbines in X-axis')
sl.declare_arg('ny', default_value = 1, description = 'how many turbines in Y-axis')

sl.declare_arg('x', default_value = 0., description = 'X-position of first turbine')
sl.declare_arg('y', default_value = 0., description = 'Y-position of first turbine')
sl.declare_arg('yaw', default_value = 0., description = 'Yaw of turbines')
sl.declare_arg('scale', default_value = 20., description = 'Distances of turbines')
sl.declare_arg('velocity', default_value = -2., description = 'Velocity of turbines')

sl.declare_arg('gz_gui', True)

sl.declare_arg('gz', True)
sl.declare_arg('spawn', True)


def launch_setup():
    resources_package = 'drone_sim_environment'

    # Make path to resources dir without last package_name fragment.
    path_to_share_dir_clipped = ''.join(
        get_package_share_directory(resources_package).rsplit('/' + resources_package, 1))

    print(path_to_share_dir_clipped)
    # Gazebo hint for resources.
    os.environ['GZ_SIM_RESOURCE_PATH'] = path_to_share_dir_clipped

    # Ensure `SDF_PATH` is populated since `sdformat_urdf` uses this rather

    # than `GZ_SIM_RESOURCE_PATH` to locate resources.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    if sl.arg('gz'):
        gz_args = '-r'
        if not sl.arg('gz_gui'):
            gz_args += ' -s'
        sl.gz_launch(sl.find('drone_sim_environment', 'drone_sim_environment_world.sdf'), gz_args)

    if sl.arg('spawn'):
        ns = 'farm'
        with sl.group(ns=ns):
            # run RSP with given parameters
            sl.robot_state_publisher('drone_sim_environment', 'farm.xacro',
                                            xacro_args=sl.arg_map('x', 'y', 'yaw', 'nx', 'ny', 'scale', 'velocity'))

            # spawn in Gazebo
            sl.spawn_gz_model(ns)

            # joint_state bridge
            gz_js_topic = GazeboBridge.model_prefix(ns)+'/joint_state'
            js_bridge = GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros)
            sl.create_gz_bridge([GazeboBridge.clock(), js_bridge], 'turbine_bridge')

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
