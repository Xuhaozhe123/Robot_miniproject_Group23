from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    fsm_node = Node(
        package='maze_nav_fsm',
        executable='maze_fsm',
        name='maze_fsm',
        output='screen',
        parameters=[{'initial_state': 'LOCALIZE'}],
    )

    planner_node = Node(
        package='maze_nav_planner',
        executable='global_astar_planner',
        name='global_astar_planner',
        output='screen',
        parameters=[{'frame_id': 'map', 'soft_obstacle_threshold': 50}],
    )

    vision_node = Node(
        package='maze_nav_vision',
        executable='blue_tape_detector',
        name='blue_tape_detector',
        output='screen',
        parameters=[{'image_topic': '/camera/image_raw'}],
    )

    return LaunchDescription([
        fsm_node,
        planner_node,
        vision_node,
    ])


