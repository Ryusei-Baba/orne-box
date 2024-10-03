#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import tf2_ros
import math
import copy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import TransformStamped
from rclpy.exceptions import ParameterNotDeclaredException

class EstimatePosture(Node):
    def __init__(self):
        super().__init__('estimate_posture')

        try:
            self.scan_front_begin = self.declare_parameter("/lidar_with_mirror/scan_front_begin", math.radians(-55)).value
            self.scan_front_end = self.declare_parameter("/lidar_with_mirror/scan_front_end", math.radians(55)).value
            self.scan_right_begin = self.declare_parameter("/lidar_with_mirror/scan_right_begin", math.radians(-115)).value
            self.scan_right_end = self.declare_parameter("/lidar_with_mirror/scan_right_end", math.radians(-65)).value
            self.scan_left_begin = self.declare_parameter("/lidar_with_mirror/scan_left_begin", math.radians(65)).value
            self.scan_left_end = self.declare_parameter("/lidar_with_mirror/scan_left_end", math.radians(115)).value
        except ParameterNotDeclaredException:
            self.get_logger().info("Parameter load error")

        self.sub_scan = self.create_subscription(LaserScan, '/lidar_with_mirror_scan', self.callback, 10)
        self.pub_scan_front = self.create_publisher(LaserScan, 'front_scan', 10)
        self.pub_right_scan = self.create_publisher(LaserScan, 'right_scan', 10)
        self.pub_left_scan = self.create_publisher(LaserScan, 'left_scan', 10)
        self.lp = lg.LaserProjection()
        self.pub_pc_right = self.create_publisher(PointCloud2, "point_cloud_right", 10)
        self.pub_pc_left = self.create_publisher(PointCloud2, "point_cloud_left", 10)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.pub_transformed_pc_front = self.create_publisher(PointCloud2, "transformed_point_cloud_front", 10)

    def find_plane(self, xs, ys, zs):
        r = np.c_[xs, ys, zs]
        c = np.mean(r, axis=0)
        r0 = r - c
        u, s, v = np.linalg.svd(r0)
        nv = v[-1, :]
        ds = np.dot(r, nv)
        return np.r_[nv, -np.mean(ds)]

    def callback(self, data):
        # Trim scan data based on specified angles
        front_data = self.trim_scan_data(data, self.scan_front_begin, self.scan_front_end)
        right_data = self.trim_scan_data(data, self.scan_right_begin, self.scan_right_end)
        left_data = self.trim_scan_data(data, self.scan_left_begin, self.scan_left_end)
        right_data.header.frame_id = "lidar_with_mirror_right_link"
        left_data.header.frame_id = "lidar_with_mirror_left_link"

        # Convert to 3D position
        right_pc = self.lp.projectLaser(right_data)
        left_pc = self.lp.projectLaser(left_data)
        
        # Get transforms
        try:
            trans_right = self.tfBuffer.lookup_transform('lidar_with_mirror_center_link', right_data.header.frame_id, data.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        try:
            trans_left = self.tfBuffer.lookup_transform('lidar_with_mirror_center_link', left_data.header.frame_id, data.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        try:
            trans = self.tfBuffer.lookup_transform('odom', 'lidar_with_mirror_center_link', data.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        
        # Transform point clouds
        right_pc_base = do_transform_cloud(right_pc, trans_right)
        left_pc_base = do_transform_cloud(left_pc, trans_left)
        self.pub_pc_right.publish(right_pc_base)
        self.pub_pc_left.publish(left_pc_base)

        # Calculate plane from point clouds
        pc_x, pc_y, pc_z = [], [], []
        for p in pc2.read_points(right_pc_base, skip_nans=True, field_names=("x", "y", "z")):
            pc_x.append(p[0])
            pc_y.append(p[1])
            pc_z.append(p[2])
        for p in pc2.read_points(left_pc_base, skip_nans=True, field_names=("x", "y", "z")):
            pc_x.append(p[0])
            pc_y.append(p[1])
            pc_z.append(p[2])
        coef = self.find_plane(pc_x, pc_y, pc_z)

        # Calculate LiDAR coordinates from the plane
        br = tf2_ros.TransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = data.header.stamp
        t.header.frame_id = "lidar_with_mirror_prismatic_link"
        t.child_frame_id = "lidar_with_mirror_estimated_link"
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = coef[3] / coef[2] - 0.34
        roll = math.atan(coef[1] / coef[2])
        pitch = math.atan(-coef[0] / coef[2] * math.cos(roll))
        quaternion = trans.transform.rotation
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        self.get_logger().info(f"estimated, {roll}, {pitch}, ground truth, {e[0]}, {e[1]}")
        q = tf.transformations.quaternion_from_euler(-roll, pitch + math.pi / 9, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)

        # Detect front LiDAR position from the coordinates
        front_pc = self.lp.projectLaser(front_data)
        front_pc.header.frame_id = "lidar_with_mirror_estimated_link"
        self.pub_transformed_pc_front.publish(front_pc)

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

def main(args=None):
    rclpy.init(args=args)
    node = EstimatePosture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
