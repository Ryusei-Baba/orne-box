#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import tf_transformations
import math
import copy
import numpy as np
from rclpy.exceptions import ParameterNotDeclaredException
import sensor_msgs_py.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile

class CalibrateMirrorPose(Node):
    def __init__(self):
        super().__init__('calibrate_mirror_angle')

        qos_profile = QoSProfile(depth=10)

        # Declare and get parameters
        try:
            self.declare_parameter("/lidar_with_mirror/mirror_distance", 0.1)
            self.declare_parameter("/lidar_with_mirror/mirror_roll_angle", math.pi / 4)
            self.declare_parameter("/lidar_with_mirror/scan_front_begin", -0.70)
            self.declare_parameter("/lidar_with_mirror/scan_front_end", 0.70)
            self.declare_parameter("/lidar_with_mirror/scan_left_begin", -2.00)
            self.declare_parameter("/lidar_with_mirror/scan_left_end", -1.05)
            self.declare_parameter("/lidar_with_mirror/scan_right_begin", 1.14)
            self.declare_parameter("/lidar_with_mirror/scan_right_end", 2.09)
            self.declare_parameter("/lidar_with_mirror/obstacle_height", 0.10)

            self.mirror_distance = self.get_parameter("/lidar_with_mirror/mirror_distance").get_parameter_value().double_value
            self.mirror_roll_angle = self.get_parameter("/lidar_with_mirror/mirror_roll_angle").get_parameter_value().double_value
            self.scan_front_begin = self.get_parameter("/lidar_with_mirror/scan_front_begin").get_parameter_value().double_value
            self.scan_front_end = self.get_parameter("/lidar_with_mirror/scan_front_end").get_parameter_value().double_value
            self.scan_left_begin = self.get_parameter("/lidar_with_mirror/scan_left_begin").get_parameter_value().double_value
            self.scan_left_end = self.get_parameter("/lidar_with_mirror/scan_left_end").get_parameter_value().double_value
            self.scan_right_begin = self.get_parameter("/lidar_with_mirror/scan_right_begin").get_parameter_value().double_value
            self.scan_right_end = self.get_parameter("/lidar_with_mirror/scan_right_end").get_parameter_value().double_value
            self.obstacle_height = self.get_parameter("/lidar_with_mirror/obstacle_height").get_parameter_value().double_value

        except ParameterNotDeclaredException:
            self.get_logger().info("Parameter load error")

        # Initialize publishers and subscribers
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.callback_scan, qos_profile)
        self.pub_scan_front = self.create_publisher(LaserScan, 'front_scan', qos_profile)
        self.pub_right_scan = self.create_publisher(LaserScan, 'right_scan', qos_profile)
        self.pub_left_scan = self.create_publisher(LaserScan, 'left_scan', qos_profile)
        self.marker_pub = self.create_publisher(Marker, "marker_pub", qos_profile)

        # Initialize marker
        self.marker = Marker()
        self.marker.header.frame_id = "laser"
        self.marker.ns = "basic_shapes"
        self.marker.id = 0
        self.marker.action = Marker.ADD
        self.marker.type = Marker.LINE_LIST
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.scale.x = 0.01
        self.marker.scale.y = 0.01
        self.marker.scale.z = 0.01
        self.marker.pose.orientation.w = 1.0

        # Laser projection and other setup
        self.lp = lg.LaserProjection()

    def trim_scan_data(self, data, start_angle_rad, end_angle_rad):
        trim_data = copy.deepcopy(data)
        start_angle_rad = max(start_angle_rad, data.angle_min)
        end_angle_rad = min(end_angle_rad, data.angle_max)
        start = int((start_angle_rad - data.angle_min) / data.angle_increment)
        end = int((end_angle_rad - data.angle_min) / data.angle_increment)
        trim_data.ranges = data.ranges[start:end]
        trim_data.intensities = data.intensities[start:end]
        trim_data.angle_min = start_angle_rad
        trim_data.angle_max = end_angle_rad
        return trim_data

    def obstacle_detect(self, list_x, list_y, a, b, threshold):
        res_x, res_y = [], []
        for x, y in zip(list_x, list_y):
            if self.dist_from_line(x, y, a, b) > threshold and math.sqrt(x ** 2 + y ** 2) > 0.1:
                res_x.append(x)
                res_y.append(y)
        if len(res_x) >= 4:
            del res_x[0:1]
            del res_y[0:1]
            del res_x[-1]
            del res_x[-1]
            del res_y[-1]
            del res_y[-1]
        return res_x, res_y

    def callback_scan(self, data):
        # Trim and process scan data
        front_data = self.trim_scan_data(data, self.scan_front_begin, self.scan_front_end)
        left_data = self.trim_scan_data(data, self.scan_left_begin, self.scan_left_end)
        right_data = self.trim_scan_data(data, self.scan_right_begin, self.scan_right_end)

        # Publish scan data
        self.pub_scan_front.publish(front_data)
        self.pub_left_scan.publish(left_data)
        self.pub_right_scan.publish(right_data)

        # Further processing omitted for brevity

if __name__ == '__main__':
    rclpy.init()
    node = CalibrateMirrorPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
