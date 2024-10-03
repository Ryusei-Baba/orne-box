#include "orne_box_lidar_with_mirror/convert_pointcloud.h"  // ヘッダーファイルをインクルード

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // ROS2ノードの初期化
    rclcpp::spin(std::make_shared<ConvertPointCloud>()); // ノードのスピン
    rclcpp::shutdown(); // ROS2のシャットダウン
    return 0;
}
