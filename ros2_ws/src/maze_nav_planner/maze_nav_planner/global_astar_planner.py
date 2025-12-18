import math
from typing import List, Tuple, Optional

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header, Bool


GridIndex = Tuple[int, int]


class AStarPlanner(Node):
    def __init__(self) -> None:
        super().__init__('global_astar_planner')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('soft_obstacle_threshold', 50)

        self.frame_id: str = self.get_parameter('frame_id').get_parameter_value().string_value
        self.soft_obstacle_threshold: int = (
            self.get_parameter('soft_obstacle_threshold').get_parameter_value().integer_value
        )

        self.allow_soft_obstacles: bool = False

        self.map: Optional[OccupancyGrid] = None
        self.map_array: Optional[np.ndarray] = None

        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10
        )
        self.start_sub = self.create_subscription(
            PoseStamped, 'global_start', self.start_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, 'global_goal', self.goal_callback, 10
        )

        self.soft_obstacles_sub = self.create_subscription(
            Bool, 'maze_nav/allow_soft_obstacles', self.soft_obstacles_callback, 10
        )

        self.path_pub = self.create_publisher(Path, 'global_plan', 10)

        self.start: Optional[PoseStamped] = None
        self.goal: Optional[PoseStamped] = None

    def soft_obstacles_callback(self, msg: Bool) -> None:
        self.allow_soft_obstacles = msg.data

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map = msg
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape((height, width))
        self.map_array = data

    def start_callback(self, msg: PoseStamped) -> None:
        self.start = msg
        self.try_plan()

    def goal_callback(self, msg: PoseStamped) -> None:
        self.goal = msg
        self.try_plan()

    def try_plan(self) -> None:
        if self.map is None or self.map_array is None:
            return
        if self.start is None or self.goal is None:
            return

        start_idx = self.world_to_map(self.start.pose.position.x,
                                      self.start.pose.position.y)
        goal_idx = self.world_to_map(self.goal.pose.position.x,
                                     self.goal.pose.position.y)
        if start_idx is None or goal_idx is None:
            return

        path_indices = self.astar(start_idx, goal_idx)
        if not path_indices:
            return

        path_msg = self.indices_to_path(path_indices)
        self.path_pub.publish(path_msg)

    def world_to_map(self, x: float, y: float) -> Optional[GridIndex]:
        if self.map is None:
            return None
        origin = self.map.info.origin.position
        resolution = self.map.info.resolution

        mx = int((x - origin.x) / resolution)
        my = int((y - origin.y) / resolution)

        if 0 <= mx < self.map.info.width and 0 <= my < self.map.info.height:
            return mx, my
        return None

    def map_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        assert self.map is not None
        origin = self.map.info.origin.position
        resolution = self.map.info.resolution
        wx = origin.x + (ix + 0.5) * resolution
        wy = origin.y + (iy + 0.5) * resolution
        return wx, wy

    def is_occupied(self, ix: int, iy: int) -> bool:
        assert self.map_array is not None
        value = self.map_array[iy, ix]

        if value == -1:
            return True

        if self.allow_soft_obstacles:
            return value >= 100

        if value >= self.soft_obstacle_threshold:
            return True
        return False

    def astar(self, start: GridIndex, goal: GridIndex) -> List[GridIndex]:
        assert self.map_array is not None
        height, width = self.map_array.shape

        open_set: List[GridIndex] = [start]
        came_from: dict[GridIndex, GridIndex] = {}

        g_score = {start: 0.0}
        f_score = {start: self.heuristic(start, goal)}

        visited = np.zeros_like(self.map_array, dtype=bool)

        while open_set:
            current = min(open_set, key=lambda idx: f_score.get(idx, math.inf))

            if current == goal:
                return self.reconstruct_path(came_from, current)

            open_set.remove(current)
            visited[current[1], current[0]] = True

            for nx, ny in self.neighbors(current, width, height):
                if visited[ny, nx]:
                    continue
                if self.is_occupied(nx, ny):
                    continue

                tentative_g = g_score[current] + self.cost_between(current, (nx, ny))
                if tentative_g < g_score.get((nx, ny), math.inf):
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = tentative_g
                    f_score[(nx, ny)] = tentative_g + self.heuristic((nx, ny), goal)
                    if (nx, ny) not in open_set:
                        open_set.append((nx, ny))

        return []

    @staticmethod
    def heuristic(a: GridIndex, b: GridIndex) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx, dy)

    @staticmethod
    def cost_between(a: GridIndex, b: GridIndex) -> float:
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        if dx + dy == 1:
            return 1.0
        return math.sqrt(2.0)

    @staticmethod
    def neighbors(idx: GridIndex, width: int, height: int) -> List[GridIndex]:
        x, y = idx
        result: List[GridIndex] = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx = x + dx
                ny = y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    result.append((nx, ny))
        return result

    def indices_to_path(self, indices: List[GridIndex]) -> Path:
        assert self.map is not None
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        path = Path()
        path.header = header
        poses: List[PoseStamped] = []
        for ix, iy in indices:
            wx, wy = self.map_to_world(ix, iy)
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            poses.append(pose)
        path.poses = poses
        return path


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


