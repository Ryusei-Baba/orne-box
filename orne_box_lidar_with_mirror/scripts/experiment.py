#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class MoveLidar(Node):
    def __init__(self):
        super().__init__('move_lidar_node')
        
        # Publisherの初期化
        self.lidar_with_mirror_mirror1_prismatic_pub  = self.create_publisher(Float64, '/lidar_with_mirror_mirror1_prismatic_controller/command', 10)
        self.lidar_with_mirror_mirror2_prismatic_pub  = self.create_publisher(Float64, '/lidar_with_mirror_mirror2_prismatic_controller/command', 10)
        self.lidar_with_mirror_mirror1_roll_pub = self.create_publisher(Float64, '/lidar_with_mirror_mirror1_roll_controller/command', 10)
        self.lidar_with_mirror_mirror2_roll_pub = self.create_publisher(Float64, '/lidar_with_mirror_mirror2_roll_controller/command', 10)
        self.lidar_with_mirror_mirror1_prismatic2_pub = self.create_publisher(Float64, '/lidar_with_mirror_mirror1_prismatic2_controller/command', 10)
        self.lidar_with_mirror_mirror2_prismatic2_pub = self.create_publisher(Float64, '/lidar_with_mirror_mirror2_prismatic2_controller/command', 10)
        self.lidar_with_mirror_prismatic_pub    = self.create_publisher(Float64, '/lidar_with_mirror_prismatic_controller/command', 10)
        self.lidar_with_mirror_pitch_pub        = self.create_publisher(Float64, '/lidar_with_mirror_pitch_controller/command', 10)
        self.lidar_with_mirror_roll_pub         = self.create_publisher(Float64, '/lidar_with_mirror_roll_controller/command', 10)
        self.caster_front_pub                   = self.create_publisher(Float64, '/caster_front_controller/command', 10)
        self.wheel_hinge_pub                    = self.create_publisher(Float64, '/wheel_hinge_controller/command', 10)

        # 初期値の設定
        time.sleep(0.5)
        self.lidar_with_mirror_mirror1_prismatic_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_mirror2_prismatic_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_mirror1_roll_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_mirror2_roll_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_mirror1_prismatic2_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_mirror2_prismatic2_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_prismatic_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_pitch_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_roll_pub.publish(Float64(data=0.0))
        self.caster_front_pub.publish(Float64(data=0.0))
        self.wheel_hinge_pub.publish(Float64(data=0.0))

        self.seq_no = 1
        self.prev_seq_no = -1

    def loop(self):
        if self.seq_no != self.prev_seq_no:
            self.is_first = True
        else:
            self.is_first = False
        self.prev_seq_no = self.seq_no

        if self.seq_no == 0:
            if self.is_first:
                self.angle_deg = -math.pi / 30 - math.pi / 30 / 5
            self.angle_deg += math.pi / 30 / 5
            self.lidar_with_mirror_roll_pub.publish(Float64(data=self.angle_deg))
            self.get_logger().info(f"roll angle: {self.angle_deg}")
            if self.angle_deg >= math.pi / 30:
                self.lidar_with_mirror_roll_pub.publish(Float64(data=0.0))
                self.seq_no = 1
        elif self.seq_no == 1:
            if self.is_first:
                self.angle_deg = -math.pi / 18 - math.pi / 18 / 5
            self.angle_deg += math.pi / 18 / 5
            self.lidar_with_mirror_pitch_pub.publish(Float64(data=self.angle_deg))
            self.get_logger().info(f"pitch angle: {self.angle_deg}")
            if self.angle_deg >= math.pi / 18:
                self.lidar_with_mirror_pitch_pub.publish(Float64(data=0.0))
                self.seq_no = 2
        elif self.seq_no == 2:
            if self.is_first:
                self.height = -0.05
            self.height += 0.01
            self.lidar_with_mirror_prismatic_pub.publish(Float64(data=self.height))
            self.get_logger().info(f"height: {self.height}")
            if self.height >= 0.05:
                self.lidar_with_mirror_prismatic_pub.publish(Float64(data=0.0))
                self.seq_no = 3
        elif self.seq_no == 3:
            if self.is_first:
                self.position = 0.0
            self.position -= 0.01
            self.lidar_with_mirror_mirror1_prismatic_pub.publish(Float64(data=self.position))
            self.lidar_with_mirror_mirror1_prismatic2_pub.publish(Float64(data=self.position))
            self.lidar_with_mirror_mirror2_prismatic_pub.publish(Float64(data=self.position))
            self.lidar_with_mirror_mirror2_prismatic2_pub.publish(Float64(data=self.position))
            self.get_logger().info(f"mirror distance: {self.position}")
            if self.position <= -0.1:
                self.lidar_with_mirror_mirror1_prismatic_pub.publish(Float64(data=0.0))
                self.lidar_with_mirror_mirror1_prismatic2_pub.publish(Float64(data=0.0))
                self.lidar_with_mirror_mirror2_prismatic_pub.publish(Float64(data=0.0))
                self.lidar_with_mirror_mirror2_prismatic2_pub.publish(Float64(data=0.0))
                self.seq_no = 4
        elif self.seq_no == 4:
            if self.is_first:
                self.angle_deg = -math.pi / 16
            self.angle_deg += math.pi / 16 / 5
            self.lidar_with_mirror_mirror1_roll_pub.publish(Float64(data=self.angle_deg))
            self.lidar_with_mirror_mirror2_roll_pub.publish(Float64(data=self.angle_deg))
            self.get_logger().info(f"mirror1_2_roll: {self.angle_deg}")
            if self.angle_deg >= math.pi / 16:
                self.lidar_with_mirror_mirror1_roll_pub.publish(Float64(data=0.0))
                self.lidar_with_mirror_mirror2_roll_pub.publish(Float64(data=0.0))
                self.seq_no += 100
        elif self.seq_no == 10:
            if self.is_first:
                self.height = -0.05
            self.height += 0.01
            self.caster_front_pub.publish(Float64(data=self.height))
            self.get_logger().info(f"caster: {self.height}")
            if self.height >= 0.05:
                self.seq_no = 100
        elif self.seq_no == 100:
            return False
        return True

def main(args=None):
    rclpy.init(args=args)
    ml = MoveLidar()

    DURATION = 2
    rate = ml.create_rate(1.0 / DURATION)

    while rclpy.ok():
        if not ml.loop():
            break
        rate.sleep()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
