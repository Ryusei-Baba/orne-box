#!/usr/bin/env python3
from __future__ import print_function
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class DemoChangeAngle(Node):
    def __init__(self):
        super().__init__('demo_change_angle')
        self.lidar_with_mirror_mirror1_prismatic_pub  = self.create_publisher(Float64, '/lidar_with_mirror_mirror1_prismatic_controller/command', 1)
        self.lidar_with_mirror_mirror2_prismatic_pub  = self.create_publisher(Float64, '/lidar_with_mirror_mirror2_prismatic_controller/command', 1)
        self.lidar_with_mirror_mirror1_yaw_pub        = self.create_publisher(Float64, '/lidar_with_mirror_mirror1_yaw_controller/command', 1)
        self.lidar_with_mirror_mirror2_yaw_pub        = self.create_publisher(Float64, '/lidar_with_mirror_mirror2_yaw_controller/command', 1)
        self.lidar_with_mirror_mirror1_roll_pub       = self.create_publisher(Float64, '/lidar_with_mirror_mirror1_roll_controller/command', 1)
        self.lidar_with_mirror_mirror2_roll_pub       = self.create_publisher(Float64, '/lidar_with_mirror_mirror2_roll_controller/command', 1)
        self.lidar_with_mirror_mirror1_prismatic2_pub = self.create_publisher(Float64, '/lidar_with_mirror_mirror1_prismatic2_controller/command', 1)
        self.lidar_with_mirror_mirror2_prismatic2_pub = self.create_publisher(Float64, '/lidar_with_mirror_mirror2_prismatic2_controller/command', 1)
        self.lidar_with_mirror_prismatic_pub          = self.create_publisher(Float64, '/lidar_with_mirror_prismatic_controller/command', 1)
        self.lidar_with_mirror_pitch_pub              = self.create_publisher(Float64, '/lidar_with_mirror_pitch_controller/command', 1)
        self.lidar_with_mirror_roll_pub               = self.create_publisher(Float64, '/lidar_with_mirror_roll_controller/command', 1)
        self.caster_front_pub                         = self.create_publisher(Float64, '/caster_front_controller/command', 1)
        self.wheel_hinge_pub                          = self.create_publisher(Float64, '/wheel_hinge_controller/command', 1)
        
        time.sleep(0.5)
        self.publish_initial_commands()

        self.seq_no = 0
        self.prev_seq_no = -1

    def publish_initial_commands(self):
        self.lidar_with_mirror_mirror1_prismatic_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_mirror2_prismatic_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_mirror1_yaw_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_mirror2_yaw_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_mirror1_roll_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_mirror2_roll_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_mirror1_prismatic2_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_mirror2_prismatic2_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_prismatic_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_pitch_pub.publish(Float64(data=0.0))
        self.lidar_with_mirror_roll_pub.publish(Float64(data=0.0))
        self.caster_front_pub.publish(Float64(data=0.0))
        self.wheel_hinge_pub.publish(Float64(data=0.0))

    def loop(self):
        if self.seq_no != self.prev_seq_no:
            self.is_first = True
        else:
            self.is_first = False
        self.prev_seq_no = self.seq_no

        if self.seq_no == 0:
            if self.is_first:
                self.angle_deg = 0.0
            self.angle_deg -= math.pi / 4 / 50
            self.lidar_with_mirror_mirror1_roll_pub.publish(Float64(data=self.angle_deg))
            self.lidar_with_mirror_mirror2_roll_pub.publish(Float64(data=self.angle_deg))
            print("mirror1_2_roll: " + str(self.angle_deg))
            if self.angle_deg <= -math.pi / 4:
                self.seq_no += 1

        elif self.seq_no == 1:
            self.angle_deg += math.pi / 4 / 50
            self.lidar_with_mirror_mirror1_roll_pub.publish(Float64(data=self.angle_deg))
            self.lidar_with_mirror_mirror2_roll_pub.publish(Float64(data=self.angle_deg))
            print("mirror1_2_roll: " + str(self.angle_deg))
            if self.angle_deg >= 0.0:
                self.seq_no += 1

        elif self.seq_no == 2:
            self.angle_deg -= math.pi / 4 / 50
            self.lidar_with_mirror_mirror1_yaw_pub.publish(Float64(data=self.angle_deg))
            self.lidar_with_mirror_mirror2_yaw_pub.publish(Float64(data=self.angle_deg))
            print("mirror1_2_yaw: " + str(self.angle_deg))
            if self.angle_deg <= -math.pi / 4:
                self.seq_no += 1

        elif self.seq_no == 3:
            self.angle_deg += math.pi / 4 / 50
            self.lidar_with_mirror_mirror1_yaw_pub.publish(Float64(data=self.angle_deg))
            self.lidar_with_mirror_mirror2_yaw_pub.publish(Float64(data=self.angle_deg))
            print("mirror1_2_yaw: " + str(self.angle_deg))
            if self.angle_deg >= 0.0:
                self.seq_no += 1

        elif self.seq_no == 4:
            if self.is_first:
                self.position = 0.0
            self.position += 0.01
            self.lidar_with_mirror_mirror1_prismatic_pub.publish(Float64(data=self.position))
            self.lidar_with_mirror_mirror1_prismatic2_pub.publish(Float64(data=self.position))
            self.lidar_with_mirror_mirror2_prismatic_pub.publish(Float64(data=self.position))
            self.lidar_with_mirror_mirror2_prismatic2_pub.publish(Float64(data=self.position))
            print("mirror1_2_prismatic: " + str(self.position))
            if self.position >= 0.2:
                self.seq_no += 1

        elif self.seq_no == 5:
            self.position -= 0.01
            self.lidar_with_mirror_mirror1_prismatic_pub.publish(Float64(data=self.position))
            self.lidar_with_mirror_mirror1_prismatic2_pub.publish(Float64(data=self.position))
            self.lidar_with_mirror_mirror2_prismatic_pub.publish(Float64(data=self.position))
            self.lidar_with_mirror_mirror2_prismatic2_pub.publish(Float64(data=self.position))
            print("mirror1_2_prismatic: " + str(self.position))
            if self.position <= 0.0:
                self.seq_no += 1

        elif self.seq_no == 6:
            if self.is_first:
                self.angle_deg = 0.0
            self.angle_deg += math.pi / 9 / 50
            self.lidar_with_mirror_pitch_pub.publish(Float64(data=self.angle_deg))
            print("pitch: " + str(self.angle_deg))
            if self.angle_deg >= math.pi / 9:
                self.seq_no += 1

        elif self.seq_no == 7:
            self.angle_deg -= math.pi / 9 / 50
            self.lidar_with_mirror_pitch_pub.publish(Float64(data=self.angle_deg))
            print("pitch: " + str(self.angle_deg))
            if self.angle_deg <= -math.pi / 18:
                self.seq_no += 1

        elif self.seq_no == 8:
            self.angle_deg += math.pi / 9 / 50
            self.lidar_with_mirror_pitch_pub.publish(Float64(data=self.angle_deg))
            print("pitch: " + str(self.angle_deg))
            if self.angle_deg >= 0.0:
                self.seq_no += 1

        elif self.seq_no == 9:
            if self.is_first:
                self.height = 0.0
            self.height += 0.1 / 25
            self.lidar_with_mirror_prismatic_pub.publish(Float64(data=self.height))
            print("prismatic: " + str(self.height))
            if self.height >= 0.1:
                self.seq_no += 1

        elif self.seq_no == 10:
            self.height -= 0.1 / 25
            self.lidar_with_mirror_prismatic_pub.publish(Float64(data=self.height))
            print("prismatic: " + str(self.height))
            if self.height <= 0.0:
                self.seq_no += 1

        elif self.seq_no == 11:
            if self.is_first:
                self.angle_deg = 0.0
            self.angle_deg += math.pi / 10 / 25
            self.lidar_with_mirror_roll_pub.publish(Float64(data=self.angle_deg))
            print("roll: " + str(self.angle_deg))
            if self.angle_deg >= math.pi / 10:
                self.seq_no += 1

        elif self.seq_no == 12:
            self.angle_deg -= math.pi / 10 / 25
            self.lidar_with_mirror_roll_pub.publish(Float64(data=self.angle_deg))
            print("roll: " + str(self.angle_deg))
            if self.angle_deg <= -math.pi / 10:
                self.seq_no += 1

        elif self.seq_no == 13:
            self.angle_deg += math.pi / 10 / 25
            self.lidar_with_mirror_roll_pub.publish(Float64(data=self.angle_deg))
            print("roll: " + str(self.angle_deg))
            if self.angle_deg >= 0:
                self.seq_no += 1

        elif self.seq_no == 14:
            if self.is_first:
                self.height = 0.0
            self.height -= 0.2 / 200
            self.caster_front_pub.publish(Float64(data=self.height))
            print("caster: " + str(self.height))
            if self.height <= -0.05:
                self.seq_no += 1

        elif self.seq_no == 15:
            self.height += 0.2 / 200
            self.caster_front_pub.publish(Float64(data=self.height))
            print("caster: " + str(self.height))
            if self.height >= 0.05:
                self.seq_no += 1

        elif self.seq_no == 16:
            self.height -= 0.2 / 200
            self.caster_front_pub.publish(Float64(data=self.height))
            print("caster: " + str(self.height))
            if self.height <= 0.0:
                self.seq_no += 1

        elif self.seq_no == 17:
            if self.is_first:
                self.angle_deg = 0.0
            self.angle_deg += math.pi / 18 / 50
            self.wheel_hinge_pub.publish(Float64(data=self.angle_deg))
            print("wheel_hinge: " + str(self.angle_deg))
            if self.angle_deg >= math.pi / 18:
                self.seq_no += 1

        elif self.seq_no == 18:
            self.angle_deg -= math.pi / 18 / 50
            self.wheel_hinge_pub.publish(Float64(data=self.angle_deg))
            print("wheel_hinge: " + str(self.angle_deg))
            if self.angle_deg <= -math.pi / 18:
                self.seq_no += 1

        elif self.seq_no == 19:
            self.angle_deg += math.pi / 18 / 50
            self.wheel_hinge_pub.publish(Float64(data=self.angle_deg))
            print("wheel_hinge: " + str(self.angle_deg))
            if self.angle_deg >= 0:
                self.seq_no += 1

        elif self.seq_no == 20:
            return False
        return True

def main(args=None):
    rclpy.init(args=args)
    dca = DemoChangeAngle()
    duration = 0.05
    r = dca.create_rate(1 / duration)

    while rclpy.ok():
        if not dca.loop():
            break
        r.sleep()

    dca.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
