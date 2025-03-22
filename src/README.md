# Simulation of a world with floating wind turbines
## Spawning a wind turbine as SDF model

The following command will spawn the wind farm with joint state publisher:

`ros2 launch drone_sim_environment turbine_launch.py`

## Spawning a wind turbine as SDF model and running PX4 simulation:
`ros2 launch drone_sim_environment wind_px4.launch.py`

parameters:

- `nx, ny`: number of turbines in x and y directions (default 1, 1)
- `x, y`: position of the first turbine (default 0, 0)
- `scale`: distance between two turbines (default 200)
- `yaw`: orientation of the farm (default 0)
- `velocity`: angular velocity of the turbines (default -15)

## Running ORB_SLAM3_ROS2
First, run ROS2 Node to convert image to mono8:

`cd ~/drone_autopilot/video_repub`
`ros2 run video_repub repub`

Now run ORB_SLAM3:

`ros2 run orbslam3 mono /home/customuser/drone_autopilot/ros2_orb_src/orbslam3_ros2/vocabulary/ORBvoc.txt /home/customuser/drone_autopilot/ros2_orb_src/orbslam3_ros2/config/monocular/TUM1.yaml
` 
