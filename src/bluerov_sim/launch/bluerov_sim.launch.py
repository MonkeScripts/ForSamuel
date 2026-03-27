import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    pkg_bluerov_sim = get_package_share_directory("bluerov_sim")
    ardusub_params_file = os.path.join(pkg_bluerov_sim, "config", "ardusub.parm")
    mavros_params_file = os.path.join(
        pkg_bluerov_sim, "mavros_params", "sim_mavros_params.yaml"
    )
    bluerov_gz_bridge_config_file = os.path.join(
        pkg_bluerov_sim, "config", "bluerov_gz_bridge.yaml"
    )

    paused = LaunchConfiguration("paused")
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    debug = LaunchConfiguration("debug")
    headless = LaunchConfiguration("headless")
    verbose = LaunchConfiguration("verbose")
    namespace = LaunchConfiguration("namespace")
    world_name = LaunchConfiguration("world_name")
    launch_ardusub = LaunchConfiguration("ardusub")
    launch_mavros = LaunchConfiguration("mavros")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    # World files are provided by bb_worlds (via GZ_SIM_RESOURCE_PATH colcon hook)
    # and bluerov2_gz. Pass the filename directly; Gazebo resolves via the resource path.
    if world_name.perform(context) != "empty.sdf":
        w_name = world_name.perform(context)
        world_filename = f"{w_name}.world"
        gz_args = world_filename
    else:
        world_filename = None
        gz_args = world_name.perform(context)

    if headless.perform(context) == "true":
        gz_args += " -s"
    if paused.perform(context) == "false":
        gz_args += " -r"
    if debug.perform(context) == "true":
        gz_args += f" -v {verbose.perform(context)}"

    # Attempt to parse spherical coordinates from the world file for ArduSub --home.
    # Searches GZ_SIM_RESOURCE_PATH directories; falls back to defaults if not found.
    home_lat = 33.810313
    home_lon = -118.393867
    home_alt = 0.0
    home_heading = 0.0
    if world_filename:
        resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
        search_dirs = resource_path.split(":") if resource_path else []
        world_full_path = None
        for d in search_dirs:
            candidate = os.path.join(d, world_filename)
            if os.path.exists(candidate):
                world_full_path = candidate
                break
        if world_full_path:
            print(f"Parsing world file for coordinates: {world_full_path}")
            try:
                tree = ET.parse(world_full_path)
                root = tree.getroot()
                world_elem = root.find("world")
                if world_elem is not None:
                    sc = world_elem.find("spherical_coordinates")
                    if sc is not None:
                        lat_elem = sc.find("latitude_deg")
                        if lat_elem is not None:
                            home_lat = float(lat_elem.text)

                        lon_elem = sc.find("longitude_deg")
                        if lon_elem is not None:
                            home_lon = float(lon_elem.text)

                        elev_elem = sc.find("elevation")
                        if elev_elem is not None:
                            home_alt = float(elev_elem.text)

                        head_elem = sc.find("heading_deg")
                        if head_elem is not None:
                            home_heading = float(head_elem.text)

                print(f"[Launch] Found Coordinates: Lat={home_lat}, Lon={home_lon}")

            except Exception as e:
                print(f"XML Parsing failed: {e}. Using defaults.")
        else:
            print("World file not found in GZ_SIM_RESOURCE_PATH. Using default coordinates.")

    # Create the string for ArduSub: "Lat,Lon,Alt,Yaw"
    home_str = f"{home_lat},{home_lon},{home_alt},{home_heading}"

    # Launch ArduSub w/ SIM_JSON
    # -w: wipe eeprom
    # --home: start location (lat,lon,alt,yaw). Yaw is provided by Gazebo, so the start yaw value is ignored.
    # ardusub on $PATH via Dockerfile ENV (ardupilot/build/sitl/bin)
    ardusub_launch = ExecuteProcess(
        cmd=[
            "ardusub",
            "-S",
            "-w",
            "-M",
            "JSON",
            "--defaults",
            ardusub_params_file,
            "-I0",
            "--home",
            home_str,
        ],
        output="screen",
        condition=IfCondition(launch_ardusub),
    )
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("gz_args", gz_args),
        ],
        condition=IfCondition(gui),
    )

    description_file = PathJoinSubstitution(
        [
            FindPackageShare("bluerov_sim"),
            "models",
            "bluerov2",
            "model.sdf",
        ]
    )

    gz_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            namespace,
            "-file",
            description_file,
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-R",
            roll,
            "-P",
            pitch,
            "-Y",
            yaw,
        ],
        output="both",
        condition=IfCondition(gui),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    spawn_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawner,
            on_exit=LogInfo(msg="Robot Model Spawn Process Finished"),
        )
    )

    # Translate messages MAV <-> ROS
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        # mavros_node is actually many nodes, so we can't override the name
        # name='mavros_node',
        parameters=[mavros_params_file],
        condition=IfCondition(launch_mavros),
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_clock_bridge",
        parameters=[{"config_file": bluerov_gz_bridge_config_file}],
    )

    ground_truth_to_mavros = Node(
        package="bluerov_sim",
        executable="ground_truth_to_mavros.py",
        output="screen",
    )

    # ArduSub must be listening on UDP 9002 before the model is spawned,
    # otherwise lock_step=1 causes Gazebo to freeze waiting for a response.
    # Start ArduSub and Gazebo first, delay spawning the model.
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[gz_spawner, spawn_exit_handler, gz_bridge, ground_truth_to_mavros],
    )

    return [
        ardusub_launch,
        gz_sim_launch,
        mavros_node,
        delayed_spawn,
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "paused",
            default_value="false",
            description="Start the simulation paused",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Flag to enable the gazebo gui",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Flag to indicate whether to use simulation time",
        ),
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Flag to enable the gazebo debug flag",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Flag to enable the gazebo headless mode",
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="0",
            description="Adjust level of console verbosity",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="empty.sdf",
            description="Gazebo world file to launch (resolved via GZ_SIM_RESOURCE_PATH)",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="bluerov",
            description="Namespace",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="0.0",
            description="Initial x position",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="0.0",
            description="Initial y position",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0.0",
            description="Initial z position",
        ),
        DeclareLaunchArgument(
            "roll",
            default_value="0.0",
            description="Initial roll angle",
        ),
        DeclareLaunchArgument(
            "pitch",
            default_value="0.0",
            description="Initial pitch angle",
        ),
        DeclareLaunchArgument(
            "yaw",
            default_value="0.0",
            description="Initial yaw angle",
        ),
        DeclareLaunchArgument(
            "ardusub", default_value="true", description="Launch ArduSUB?"
        ),
        DeclareLaunchArgument(
            "mavros", default_value="true", description="Launch mavros?"
        ),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
