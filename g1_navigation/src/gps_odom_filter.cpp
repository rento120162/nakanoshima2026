#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomFilter : public rclcpp::Node
{
public:
    OdomFilter(): Node("odom_filter")
    {
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>( "/odometry/gps", 10, std::bind(&OdomFilter::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/gps_filtered", 10);

        //RCLCPP_INFO(this->get_logger(), "Odom subscriber started");
    }

private:

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

    void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 表示
        double x = -(msg->pose.pose.position.y);
        msg->pose.pose.position.y = (msg->pose.pose.position.x);
        msg->pose.pose.position.x = x;
        pub_->publish(*msg);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}