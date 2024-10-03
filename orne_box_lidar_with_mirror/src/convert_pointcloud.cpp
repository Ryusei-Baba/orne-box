#include "orne_box_lidar_with_mirror/convert_pointcloud.h"  // ヘッダーファイルをインクルード

ConvertPointCloud::ConvertPointCloud()
    : Node("convert_pointcloud_node"),
      tfBuffer_(this->get_clock()),
      tfListener_(tfBuffer_) 
{
    // パブリッシャーの作成
    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    
    // サブスクリプションの作成
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ConvertPointCloud::scanCallback, this, std::placeholders::_1));
}

void ConvertPointCloud::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
    sensor_msgs::msg::PointCloud2 cloud; // 出力の点群

    try {
        // LaserScanをPointCloud2に変換
        projector_.projectLaser(*scan, cloud, -1.0); // -1.0はカットオフ距離
        point_cloud_publisher_->publish(cloud); // 点群をパブリッシュ
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    }
}
