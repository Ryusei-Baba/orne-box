#!/usr/bin/env python3
from __future__ import print_function
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import copy

class MergeMeasuredDataNode(Node):
    def __init__(self):
        super().__init__('merge_measured_data_node')
        self.lidar_center = None
        self.lidar_left = None
        self.lidar_right = None

        self.merged_lidar_pub = self.create_publisher(LaserScan, "/lidar_with_mirror_scan", 1)
        self.lidar_with_mirror_center_sub = self.create_subscription(
            LaserScan,
            "/lidar_with_mirror_center_scan",
            self.callback_lidar_with_mirror_center,
            1
        )
        self.lidar_with_mirror_left_sub = self.create_subscription(
            LaserScan,
            "/lidar_with_mirror_left_scan",
            self.callback_lidar_with_mirror_left,
            1
        )
        self.lidar_with_mirror_right_sub = self.create_subscription(
            LaserScan,
            "/lidar_with_mirror_right_scan",
            self.callback_lidar_with_mirror_right,
            1
        )

    def callback_lidar_with_mirror_center(self, data):
        self.lidar_center = copy.deepcopy(data)
        if self.lidar_center is not None and self.lidar_left is not None and self.lidar_right is not None:
            self.merge()

    def callback_lidar_with_mirror_left(self, data):
        self.lidar_left = copy.deepcopy(data)
        if self.lidar_center is not None and self.lidar_left is not None and self.lidar_right is not None:
            self.merge()

    def callback_lidar_with_mirror_right(self, data):
        self.lidar_right = copy.deepcopy(data)
        if self.lidar_center is not None and self.lidar_left is not None and self.lidar_right is not None:
            self.merge()

    def merge(self):
        merged_lidar = copy.deepcopy(self.lidar_center)
        merged_lidar.angle_min = self.lidar_right.angle_min
        merged_lidar.angle_max = self.lidar_left.angle_max
        merged_lidar.ranges = self.lidar_right.ranges + self.lidar_center.ranges + self.lidar_left.ranges
        merged_lidar.intensities = self.lidar_right.intensities + self.lidar_center.intensities + self.lidar_left.intensities
        self.merged_lidar_pub.publish(merged_lidar)
        self.lidar_center = None
        self.lidar_left = None
        self.lidar_right = None

def main(args=None):
    rclpy.init(args=args)
    merge_measured_data_node = MergeMeasuredDataNode()

    rclpy.spin(merge_measured_data_node)

    merge_measured_data_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
