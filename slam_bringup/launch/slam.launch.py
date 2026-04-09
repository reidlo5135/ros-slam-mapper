from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def build_lifecycle_node(package_name: str, executable_name: str, node_name: str, params_file):
    return LifecycleNode(
        package=package_name,
        executable=executable_name,
        namespace="slam",
        name=node_name,
        output="screen",
        parameters=[params_file],
    )


def build_activation_chain(node: LifecycleNode):
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda action: action == node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda action: action == node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return [configure_event, activate_event]


def generate_launch_description() -> LaunchDescription:
    params_file = PathJoinSubstitution(
        [FindPackageShare("slam_bringup"), "params", "slam.yaml"]
    )

    slam_scan_matcher = build_lifecycle_node(
        "slam_scan_matcher",
        "slam_scan_matcher",
        "slam_scan_matcher",
        params_file,
    )
    slam_submap_server = build_lifecycle_node(
        "slam_submap_server",
        "slam_submap_server",
        "slam_submap_server",
        params_file,
    )
    slam_pgraph_server = build_lifecycle_node(
        "slam_pgraph_server",
        "slam_pgraph_server",
        "slam_pgraph_server",
        params_file,
    )

    entities = [
        slam_scan_matcher,
        slam_submap_server,
        slam_pgraph_server,
    ]

    for node in [slam_scan_matcher, slam_submap_server, slam_pgraph_server]:
        entities.extend(build_activation_chain(node))

    return LaunchDescription(
        entities
    )
