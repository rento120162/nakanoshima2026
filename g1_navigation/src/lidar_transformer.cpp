#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class LidarTransformer : public rclcpp::Node
{
public:
    LidarTransformer()
    : Node("rotate_180_node")
    {
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>( "/livox/lidar", 10, std::bind(&LidarTransformer::cloudCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>( "/livox/imu", 10, std::bind(&LidarTransformer::imuCallback, this, std::placeholders::_1));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>( "/livox_lidar_fixed", 10);

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/livox_imu_fixed", 10);

        RCLCPP_INFO(this->get_logger(), "LidarTransformer started");
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // --------------------------------------------------------
    // PointCloud2
    // --------------------------------------------------------
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto out = *msg;

        // roll 180°:
        // x -> x
        // y -> -y
        // z -> -z

        sensor_msgs::PointCloud2Modifier modifier(out);

        sensor_msgs::PointCloud2Iterator<float> iter_x(out, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(out, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(out, "z");

        for (; iter_x != iter_x.end();
             ++iter_x, ++iter_y, ++iter_z)
        {
            *iter_y = -*iter_y;
            *iter_z = -*iter_z;
        }

        cloud_pub_->publish(out);
    }

    // --------------------------------------------------------
    // IMU
    // --------------------------------------------------------
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto out = *msg;

        // 元の姿勢
        tf2::Quaternion q_orig(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );

        // roll 180°
        tf2::Quaternion q_rot;
        q_rot.setRPY(M_PI, 0.0, 0.0);

        // 合成
        tf2::Quaternion q_new = q_rot * q_orig;
        q_new.normalize();

        out.orientation.x = q_new.x();
        out.orientation.y = q_new.y();
        out.orientation.z = q_new.z();
        out.orientation.w = q_new.w();

        // 加速度
        out.linear_acceleration.y *= -1.0;
        out.linear_acceleration.z *= -1.0;

        // 角速度
        out.angular_velocity.y *= -1.0;
        out.angular_velocity.z *= -1.0;

        imu_pub_->publish(out);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LidarTransformer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}