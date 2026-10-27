import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_name = LaunchConfiguration("world_name").perform(context)
    world_path_arg = LaunchConfiguration("world_path").perform(context)
    gui_flag = LaunchConfiguration("gui").perform(context)
    
    world_path = ""
    if world_path_arg and os.path.isfile(world_path_arg):
        world_path = world_path_arg
    else:
        try:
            pkg_dave_worlds = get_package_share_directory("dave_worlds")
            world_file_name = f"{world_name}.world"
            world_path = os.path.join(pkg_dave_worlds, "worlds", world_file_name)
            if not os.path.isfile(world_path):
                world_path = ""
        except Exception:
            world_path = ""

    gz_args = "-r"
    if world_path:
        gz_args = f"-r {world_path}"
    
    if gui_flag.lower() != "true":
        gz_args += " -s"

    # Export the resource path so Gazebo can find our custom models
    pkg_model = get_package_share_directory("model")
    models_path = os.path.join(pkg_model, "models")
    os.environ["GZ_SIM_RESOURCE_PATH"] = models_path

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    return [gz_sim]

def generate_launch_description():
    model_share = get_package_share_directory("model")
    default_world_path = os.path.join(model_share, "worlds", "dave_ocean_waves_fixed.world")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_name",
                default_value="dave_bimanual_example",
                description="Name of the world file",
            ),
            DeclareLaunchArgument(
                "world_path",
                default_value=default_world_path,
                description="Absolute path to a world file",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Enable Gazebo GUI",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
