#ifndef LIVOX_TO_POINTCLOUD2_HPP
#define LIVOX_TO_POINTCLOUD2_HPP

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

/**
 * Converts livox_ros_driver2/CustomMsg into sensor_msgs/PointCloud2.
 *
 * Topics (relative, remap them in the launch file):
 *   subscribes: livox/lidar         (livox_ros_driver2/msg/CustomMsg)
 *   publishes:  livox/point_cloud2  (sensor_msgs/msg/PointCloud2)
 *
 * Point layout (point_step = 18 bytes, packed):
 *   x, y, z, intensity : float32 at offsets 0, 4, 8, 12
 *   tag, line          : uint8   at offsets 16, 17
 */
class LivoxToPointCloud2 : public rclcpp::Node
{
public:
  LivoxToPointCloud2();

private:
  void callback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg);

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

#endif  // LIVOX_TO_POINTCLOUD2_HPP
