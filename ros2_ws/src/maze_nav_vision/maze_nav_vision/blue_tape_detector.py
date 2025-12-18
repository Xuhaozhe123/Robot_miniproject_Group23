from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid


class BlueTapeDetector(Node):
    def __init__(self) -> None:
        super().__init__('blue_tape_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('mask_topic', 'soft_obstacle_mask')
        self.declare_parameter('width', 36)
        self.declare_parameter('height', 36)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.mask_topic = self.get_parameter('mask_topic').get_parameter_value().string_value
        self.grid_width = self.get_parameter('width').get_parameter_value().integer_value
        self.grid_height = self.get_parameter('height').get_parameter_value().integer_value

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.mask_pub = self.create_publisher(OccupancyGrid, self.mask_topic, 10)

        self.last_stamp: Optional[rclpy.time.Time] = None

    def image_callback(self, msg: Image) -> None:
        self.last_stamp = rclpy.time.Time.from_msg(msg.header.stamp)

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([90, 80, 80])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        grid = self.mask_to_grid(mask, self.grid_width, self.grid_height)
        msg_out = self.grid_to_msg(grid)
        msg_out.header = msg.header

        self.mask_pub.publish(msg_out)

    @staticmethod
    def mask_to_grid(mask: np.ndarray, width: int, height: int) -> np.ndarray:
        h, w = mask.shape
        cell_h = h // height
        cell_w = w // width

        grid = np.zeros((height, width), dtype=np.int8)
        for gy in range(height):
            for gx in range(width):
                y0 = gy * cell_h
                y1 = (gy + 1) * cell_h
                x0 = gx * cell_w
                x1 = (gx + 1) * cell_w
                cell = mask[y0:y1, x0:x1]
                ratio = float(np.count_nonzero(cell)) / float(cell.size)
                if ratio > 0.2:
                    grid[gy, gx] = 100
                else:
                    grid[gy, gx] = 0
        return grid

    def grid_to_msg(self, grid: np.ndarray) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.info.width = grid.shape[1]
        msg.info.height = grid.shape[0]
        msg.info.resolution = 0.05
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlueTapeDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


