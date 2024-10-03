#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import math
import rclpy
from rclpy.node import Node

max_bin = 100

class HoughTransformNode(Node):
    def __init__(self):
        super().__init__('hough_transform_node')

        # Precompute sin and cos values for Hough Transform
        sin_l, cos_l = [], []
        for t in np.arange(2 * math.pi / max_bin / 2, 2 * math.pi, 2 * math.pi / max_bin):
            sin_l.append(math.sin(t))
            cos_l.append(math.cos(t))
        self.sin_t, self.cos_t = np.array(sin_l), np.array(cos_l)

        # Example data
        self.list_x = np.arange(0, 2, 0.1)
        self.list_y = 2 * self.list_x + 2
        self.get_logger().info(f'List X: {self.list_x}')
        self.get_logger().info(f'List Y: {self.list_y}')

        # Run the Hough Transform
        self.hough(self.list_x, self.list_y)

    def hough(self, list_x, list_y):
        max_rho = 10
        bin = np.zeros((max_bin, max_bin))

        for x, y in zip(list_x, list_y):
            rho = x * self.cos_t + y * self.sin_t
            for t in range(max_bin):
                bin[t][int((rho[t] / max_rho * max_bin + max_bin) / 2)] += 1

        self.get_logger().info(f'Hough Bin: \n{bin}')

        max_angle = bin.argmax() // max_bin
        max_dist = bin.argmax() % max_bin

        self.get_logger().info(f'Max Angle: {max_angle}, Max Distance: {max_dist}, Count: {bin[max_angle][max_dist]}')
        self.get_logger().info(
            f"Line Equation: y = {-self.cos_t[max_angle] / self.sin_t[max_angle]}x + {1 / self.sin_t[max_angle] * (max_dist - max_bin / 2) / (max_bin / 2) * max_rho}"
        )

def main(args=None):
    rclpy.init(args=args)
    hough_transform_node = HoughTransformNode()
    rclpy.spin(hough_transform_node)
    hough_transform_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
