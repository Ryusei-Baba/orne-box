#ifndef CONVERT_POINTCLOUD_H
#define CONVERT_POINTCLOUD_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ConvertPointCloud : public rclcpp::Node {
public:
    ConvertPointCloud();  // コンストラクタの宣言
    void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan); // メンバ関数の宣言

private:
    laser_geometry::LaserProjection projector_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

#endif // CONVERT_POINTCLOUD_H
