#include "livox_to_pointcloud2.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "sensor_msgs/msg/point_field.hpp"

namespace
{
constexpr uint32_t kPointStep = 18;

// Appends one field description to the PointCloud2 message.
void addField(
  sensor_msgs::msg::PointCloud2 & cloud, const char * name, uint32_t offset, uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  cloud.fields.push_back(field);
}

// Unaligned-safe store of a trivially copyable value (point_step is 18, so
// float fields are not 4-byte aligned for every point).
template<typename T>
void store(uint8_t * destination, const T & value)
{
  std::memcpy(destination, &value, sizeof(T));
}
}  // namespace

LivoxToPointCloud2::LivoxToPointCloud2()
: Node("livox_to_pointcloud2")
{
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("livox/point_cloud2", 10);
  subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
    "livox/lidar", 10,
    std::bind(&LivoxToPointCloud2::callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(), "Converting %s -> %s",
    subscription_->get_topic_name(), publisher_->get_topic_name());
}

void LivoxToPointCloud2::callback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
{
  // point_num comes from the driver; never trust it more than the real array size,
  // otherwise a mismatch would write outside the output buffer.
  const std::size_t count = std::min<std::size_t>(msg->point_num, msg->points.size());
  if (count != msg->point_num) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "point_num (%u) differs from points.size() (%zu), using %zu",
      msg->point_num, msg->points.size(), count);
  }

  sensor_msgs::msg::PointCloud2 output;
  output.header = msg->header;

  addField(output, "x", 0, sensor_msgs::msg::PointField::FLOAT32);
  addField(output, "y", 4, sensor_msgs::msg::PointField::FLOAT32);
  addField(output, "z", 8, sensor_msgs::msg::PointField::FLOAT32);
  addField(output, "intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
  addField(output, "tag", 16, sensor_msgs::msg::PointField::UINT8);
  addField(output, "line", 17, sensor_msgs::msg::PointField::UINT8);

  output.height = 1;
  output.width = static_cast<uint32_t>(count);
  output.is_bigendian = false;
  output.is_dense = true;
  output.point_step = kPointStep;
  output.row_step = kPointStep * output.width;
  output.data.resize(static_cast<std::size_t>(output.row_step));

  uint8_t * out = output.data.data();
  for (std::size_t i = 0; i < count; ++i) {
    const auto & point = msg->points[i];
    store<float>(out + 0, point.x);
    store<float>(out + 4, point.y);
    store<float>(out + 8, point.z);
    store<float>(out + 12, static_cast<float>(point.reflectivity));
    store<uint8_t>(out + 16, point.tag);
    store<uint8_t>(out + 17, point.line);
    out += kPointStep;
  }

  publisher_->publish(output);
}
