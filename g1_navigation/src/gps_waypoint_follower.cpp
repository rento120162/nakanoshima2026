// gps_waypoint_to_follow_waypoints.cpp

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

class GPSWaypointSender : public rclcpp::Node
{
public:
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;

    struct Waypoint {
        double lat;
        double lon;
    };

    GPSWaypointSender(const std::string & yaml_path)
        : Node("gps_waypoint_sender")
    {
        loadYaml(yaml_path);

        fromll_client_ =
            this->create_client<robot_localization::srv::FromLL>("/fromLL");

        action_client_ =
            rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");

        waitForServices();

        convertAll();
    }

private:
    std::vector<Waypoint> gps_wps_;
    std::vector<geometry_msgs::msg::PoseStamped> map_wps_;

    rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr fromll_client_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;

    size_t convert_index_ = 0;

    void loadYaml(const std::string & path)
    {
        YAML::Node config = YAML::LoadFile(path);

        for (auto wp : config["waypoints"]) {
            gps_wps_.push_back({
                wp["latitude"].as<double>(),
                wp["longitude"].as<double>()
            });
        }

        RCLCPP_INFO(get_logger(), "Loaded %ld GPS waypoints", gps_wps_.size());
    }

    void waitForServices()
    {
        while (!fromll_client_->wait_for_service(1s)) {
            RCLCPP_INFO(get_logger(), "Waiting for /fromLL...");
        }

        while (!action_client_->wait_for_action_server(1s)) {
            RCLCPP_INFO(get_logger(), "Waiting for follow_waypoints...");
        }
    }

    void convertAll()
    {
        convert_index_ = 0;
        requestNext();
    }

    void requestNext()
    {
        if (convert_index_ >= gps_wps_.size()) {
            sendGoal();
            return;
        }

        auto & wp = gps_wps_[convert_index_];

        auto req = std::make_shared<robot_localization::srv::FromLL::Request>();
        req->ll_point.latitude = wp.lat;
        req->ll_point.longitude = wp.lon;
        req->ll_point.altitude = 0.0;

        fromll_client_->async_send_request(
            req,
            std::bind(&GPSWaypointSender::fromLLCallback, this, std::placeholders::_1));
    }

    void fromLLCallback(
        rclcpp::Client<robot_localization::srv::FromLL>::SharedFuture future)
    {
        auto res = future.get();

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now();

        // ←ここは環境依存（反転してるなら残す）
        pose.pose.position.x = -(res->map_point.y);
        pose.pose.position.y = (res->map_point.x);
        pose.pose.orientation.w = 1.0;

        map_wps_.push_back(pose);

        RCLCPP_INFO(get_logger(), "Converted %ld", convert_index_);

        convert_index_++;
        requestNext();
    }

    void sendGoal()
    {
        RCLCPP_INFO(get_logger(), "Sending %ld waypoints", map_wps_.size());

        for (size_t i = 0; i < map_wps_.size(); i++) {
            auto & p = map_wps_[i].pose.position;
            RCLCPP_INFO(get_logger(),
                "WP[%ld]: x=%.3f y=%.3f",
                i, p.x, p.y);
        }

        FollowWaypoints::Goal goal_msg;
        goal_msg.poses = map_wps_;

        auto options =
            rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

        options.feedback_callback =
            [](auto, auto feedback) {
                RCLCPP_INFO(rclcpp::get_logger("feedback"),
                            "Current WP: %d",
                            feedback->current_waypoint);
            };

        action_client_->async_send_goal(goal_msg, options);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cout << "usage: node waypoints.yaml\n";
        return 1;
    }

    auto node = std::make_shared<GPSWaypointSender>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
}