#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class OdomToBaseLinkBroadcaster : public rclcpp::Node
{
public:
  OdomToBaseLinkBroadcaster()
  : Node("tf_pub")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // Subscribe to the odometry topic
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "ddd/odom", 10,
      std::bind(&OdomToBaseLinkBroadcaster::handle_odom, this, std::placeholders::_1));
  }

private:
  void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transformStamped;

    // Populate the transformStamped message
    transformStamped.header.stamp = msg->header.stamp; //this->get_clock()->now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";

    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation = msg->pose.pose.orientation;

    // Send the transform
    tf_broadcaster_->sendTransform(transformStamped);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToBaseLinkBroadcaster>());
  //rclcpp::shutdown();
  return 0;
}
