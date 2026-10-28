"""Single-command real-time demo: state_publisher + 3D-EKF + SSA-UKF (v1) + SSA-UKF v2
+ friction estimator, replacing the 4 manual terminal commands in the upstream README.

Everything here talks over the same standard topic names the nodes always use
(/vehicle/sensor/..., /core/state/..., /core/ssa/...) -- there is nothing bag-specific
about this launch file. Point `bag_path` at a recorded bag directory for offline replay
at real-time (1x) rate, or omit it and point the same topic names at live vehicle
sensors later; the estimation/GUI stack does not change either way.

v2 (`ssa_estimation_v2_node_cpp`, node name SSAEstimationV2) runs by default alongside
v1, remapped onto /core/ssa_v2/... so it never collides with v1's own /core/ssa/... --
`friction_estimation_node` treats it as an optional, parallel second-opinion source: it
publishes `mu_demand_axle_evidence`/`axle_evidence_valid` on FrictionEstimate whenever
v2's debug stream is fresh, and mu_demand (the primary, EKF-only estimate) is
unaffected if v2 is disabled or not yet started. Set `use_v2:=false` to omit it.

Usage:
    ros2 launch friction_estimation_cpp friction_estimation_demo.launch.py \\
        bag_path:=/path/to/vegas24_pub

Config file locations default to this machine's existing checkout layout
(~/ros2_ws/src/3DVehicleDynamicsStateEstimation) -- override with the *_config_path
arguments if your workspace layout differs.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

STATE_ESTIMATION_REPO_DEFAULT = os.path.expanduser(
    "~/ros2_ws/src/3DVehicleDynamicsStateEstimation")


def generate_launch_description():
    # Restrict DDS discovery to this machine only. Left at its default (SUBNET on this
    # environment), FastDDS tries to discover participants over multicast across the
    # whole local network -- on at least one WSL2 setup this caused real, reproducible
    # multi-second gaps in message delivery early in a 1x bag replay (confirmed via an
    # independent `ros2 topic hz` subscriber catching the same gap), well past what the
    # 500ms IMU topic watchdog tolerates, causing state_estimation_node's safety-stop to
    # fire seconds into playback instead of at the bag's natural end. Everything in this
    # launch file runs on localhost only, so there is no reason for discovery to ever
    # leave the machine.
    discovery_range_env = SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")

    state_estimation_repo_arg = DeclareLaunchArgument(
        "state_estimation_repo", default_value=STATE_ESTIMATION_REPO_DEFAULT,
        description="Path to the 3DVehicleDynamicsStateEstimation checkout "
                    "(source of the EKF/SSA config ymls)")
    ekf_config_arg = DeclareLaunchArgument(
        "ekf_config_path",
        default_value=PathJoinSubstitution([
            LaunchConfiguration("state_estimation_repo"),
            "config", "state_estimation_cpp_3d_ekf_config.yml"]))
    ssa_config_arg = DeclareLaunchArgument(
        "ssa_config_path",
        default_value=PathJoinSubstitution([
            LaunchConfiguration("state_estimation_repo"),
            "config", "ssa_estimation_ukf_config.yml"]))
    friction_config_arg = DeclareLaunchArgument(
        "friction_config_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("friction_estimation_cpp"),
            "config", "friction_estimation_config.yml"]))
    use_v2_arg = DeclareLaunchArgument(
        "use_v2", default_value="true",
        description="Bring up ssa_estimation_v2_node_cpp alongside v1 (remapped onto "
                    "/core/ssa_v2/...) so friction_estimation_node can publish the "
                    "second, axle-force-evidence demand estimate. mu_demand itself "
                    "(EKF-only) is unaffected either way.")
    ssa_v2_config_arg = DeclareLaunchArgument(
        "ssa_v2_config_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("ssa_estimation_v2_node_cpp"),
            "config", "ssa_estimation_v2_ukf_config.yml"]))
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Replay on the bag's own clock: `ros2 bag play --clock-topics-all` and "
                    "use_sim_time on every node, so every header stamp equals the bag receive "
                    "time. This removes the wall-clock offset/jitter between a live replay and "
                    "an offline read of the same bag (needed for time-aligned evaluation against "
                    "Kistler) and makes the node watchdogs immune to WSL2 scheduling gaps.")
    bag_path_arg = DeclareLaunchArgument(
        "bag_path", default_value="",
        description="Bag directory to auto-play at real-time (1x) rate. "
                    "Leave empty to just bring the estimation stack up and feed it "
                    "live/manually-played data instead.")
    bag_start_delay_arg = DeclareLaunchArgument(
        "bag_start_delay_s", default_value="4.0",
        description="Seconds to wait before starting bag playback, so the EKF/SSA nodes "
                    "are fully up before the first messages arrive. Starting playback "
                    "against nodes still mid cold-start has caused real EKF divergence "
                    "in testing -- don't lower this without re-verifying against a "
                    "full replay.")

    # Same technique as packages_cpp/state_publisher/launch/state_publisher.py: resolve
    # the package share dir and read the URDF eagerly, since robot_description has to be
    # a plain string parameter, not a lazily-resolved substitution.
    state_publisher_share = FindPackageShare("state_publisher").find("state_publisher")
    urdf_file = os.path.join(state_publisher_share, "urdf", "urdf.xml")
    with open(urdf_file, "r") as f:
        robot_description = f.read()

    state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="TransformPublisher",
        namespace="/core/state",
        output="screen",
        parameters=[{
            "publish_frequency": 0.1,
            "ignore_timestamp": False,
            "use_tf_static": True,
            "robot_description": robot_description,
        }],
    )

    # v2 fork (state_estimation_node_v2_cpp), not upstream's state_estimation_node_cpp --
    # same node name/config/topics, only the linear-velocity outlier check is fixed (it
    # only compared a new measurement against the EKF's own current state, so once that
    # state ever drifted past the outlier threshold every subsequent correct measurement
    # also looked like an outlier and got rejected, a self-reinforcing lock observed to
    # cause 50+ second v_mps divergences in about 1/3 of repeated replays -- see
    # docs/TODO_Advisor_Action_Items.md and state_estimation_v2_cpp/package.xml).
    ekf_node = Node(
        package="state_estimation_node_v2_cpp",
        executable="state_estimation_node_v2_cpp_ekf_3d_kin",
        name="StateEstimation",
        output="screen",
        parameters=[LaunchConfiguration("ekf_config_path")],
    )

    ssa_node = Node(
        package="ssa_estimation_node_cpp",
        executable="ssa_estimation_node",
        name="SSAEstimation",
        output="screen",
        parameters=[LaunchConfiguration("ssa_config_path")],
    )

    friction_node = Node(
        package="friction_estimation_cpp",
        executable="friction_estimation_node",
        name="FrictionEstimation",
        output="screen",
        parameters=[LaunchConfiguration("friction_config_path")],
    )

    ssa_v2_node = Node(
        package="ssa_estimation_v2_node_cpp",
        executable="ssa_estimation_v2_node",
        name="SSAEstimationV2",
        output="screen",
        parameters=[LaunchConfiguration("ssa_v2_config_path")],
        remappings=[
            ("/core/ssa/odometry", "/core/ssa_v2/odometry"),
            ("/core/ssa/status", "/core/ssa_v2/status"),
        ],
        condition=IfCondition(LaunchConfiguration("use_v2")),
    )

    bag_play = TimerAction(
        period=LaunchConfiguration("bag_start_delay_s"),
        actions=[ExecuteProcess(
            cmd=["ros2", "bag", "play", LaunchConfiguration("bag_path")],
            output="screen",
        )],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("bag_path"), "' != '' and '",
             LaunchConfiguration("use_sim_time"), "' != 'true'"])),
    )
    bag_play_sim_clock = TimerAction(
        period=LaunchConfiguration("bag_start_delay_s"),
        actions=[ExecuteProcess(
            cmd=["ros2", "bag", "play", LaunchConfiguration("bag_path"), "--clock-topics-all"],
            output="screen",
        )],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("bag_path"), "' != '' and '",
             LaunchConfiguration("use_sim_time"), "' == 'true'"])),
    )
    sim_time_param = SetParameter(
        name="use_sim_time", value=True,
        condition=IfCondition(LaunchConfiguration("use_sim_time")))

    return LaunchDescription([
        discovery_range_env,
        state_estimation_repo_arg, ekf_config_arg, ssa_config_arg,
        friction_config_arg, use_v2_arg, ssa_v2_config_arg,
        use_sim_time_arg, bag_path_arg, bag_start_delay_arg, sim_time_param,
        state_publisher_node, ekf_node, ssa_node, ssa_v2_node, friction_node,
        bag_play, bag_play_sim_clock,
    ])
