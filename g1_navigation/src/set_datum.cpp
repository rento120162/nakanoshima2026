#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "robot_localization/srv/set_datum.hpp"
#include "geographic_msgs/msg/geo_pose.hpp"

class AutoDatumSetter : public rclcpp::Node
{
public:
    AutoDatumSetter() : Node("auto_datum_setter"), datum_sent_(false) 
    {
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/navsatfix", 10, std::bind(&AutoDatumSetter::gpsCallback, this, std::placeholders::_1));
        datum_client_ = this->create_client<robot_localization::srv::SetDatum>("/datum");

        RCLCPP_INFO(this->get_logger(), "Waiting for /datum ...");

        datum_client_->wait_for_service();

        RCLCPP_INFO(this->get_logger(), "Service available.");
    }

private:

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (datum_sent_) return;

        if (msg->status.status < 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid GPS fix");
            return;
        }

        auto request = std::make_shared<robot_localization::srv::SetDatum::Request>();

        geographic_msgs::msg::GeoPose geo_pose;
        geo_pose.position.latitude = msg->latitude;
        geo_pose.position.longitude = msg->longitude;
        geo_pose.position.altitude = 0.0;
        geo_pose.orientation.x = 0.0;
        geo_pose.orientation.y = 0.0;
        geo_pose.orientation.z = 0.0;
        geo_pose.orientation.w = 1.0;

        request->geo_pose = geo_pose;

        RCLCPP_INFO(
            this->get_logger(),
            "Setting datum: lat=%f lon=%f",
            msg->latitude,
            msg->longitude
        );

        auto future =
            datum_client_->async_send_request(
                request,
                std::bind(&AutoDatumSetter::responseCallback,this,std::placeholders::_1)
            );

        datum_sent_ = true;
    }

    void responseCallback(
        rclcpp::Client<robot_localization::srv::SetDatum>::SharedFuture future)
    {
        try
        {
            future.get();
            RCLCPP_INFO(this->get_logger(), "Datum set successfully");
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    bool datum_sent_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Client<robot_localization::srv::SetDatum>::SharedPtr datum_client_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoDatumSetter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}