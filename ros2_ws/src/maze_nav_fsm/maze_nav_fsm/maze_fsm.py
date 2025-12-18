from enum import Enum, auto

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool


class NavState(Enum):
    IDLE = auto()
    LOCALIZE = auto()
    NAV_TO_START = auto()
    NAV_TO_GOAL = auto()
    FINISHED = auto()


class MazeFSM(Node):
    def __init__(self) -> None:
        super().__init__('maze_fsm')

        self.declare_parameter('initial_state', 'IDLE')

        self.state = self._parse_state(
            self.get_parameter('initial_state').get_parameter_value().string_value
        )

        self.state_pub = self.create_publisher(String, 'maze_nav/state', 10)
        self.soft_pass_pub = self.create_publisher(Bool, 'maze_nav/allow_soft_obstacles', 10)

        self.localize_done_sub = self.create_subscription(
            Bool, 'maze_nav/localization_done', self.localize_done_callback, 10
        )
        self.start_reached_sub = self.create_subscription(
            Bool, 'maze_nav/start_reached', self.start_reached_callback, 10
        )
        self.goal_reached_sub = self.create_subscription(
            Bool, 'maze_nav/goal_reached', self.goal_reached_callback, 10
        )

        self.timer = self.create_timer(0.1, self.publish_state)
        self.update_costmap_mode()

    @staticmethod
    def _parse_state(name: str) -> NavState:
        try:
            return NavState[name]
        except KeyError:
            return NavState.IDLE

    def publish_state(self) -> None:
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def update_costmap_mode(self) -> None:
        msg = Bool()
        if self.state in (NavState.LOCALIZE, NavState.NAV_TO_START):
            msg.data = True
        else:
            msg.data = False
        self.soft_pass_pub.publish(msg)

    def localize_done_callback(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self.state == NavState.LOCALIZE:
            self.state = NavState.NAV_TO_START
            self.update_costmap_mode()

    def start_reached_callback(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self.state == NavState.NAV_TO_START:
            self.state = NavState.NAV_TO_GOAL
            self.update_costmap_mode()

    def goal_reached_callback(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self.state == NavState.NAV_TO_GOAL:
            self.state = NavState.FINISHED
            self.update_costmap_mode()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MazeFSM()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


