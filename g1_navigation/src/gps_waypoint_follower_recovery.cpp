
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <GeographicLib/Geodesic.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

class GPSWaypointInitializer : public rclcpp::Node
{
public:

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;

    using GoalHandleNavigateToPose =
        rclcpp_action::ClientGoalHandle<NavigateToPose>;

    struct GPSWaypoint
    {
        double lat;
        double lon;
    };

    GPSWaypointInitializer(const std::string & yaml_path)
        : Node("gps_waypoint_initializer")
    {
        loadYaml(yaml_path);

        // ============================================
        // subscriber
        // ============================================

        gps_sub_ =
            create_subscription<sensor_msgs::msg::NavSatFix>(
                "/navsatfix",
                10,
                std::bind(
                    &GPSWaypointInitializer::gpsCallback,
                    this,
                    std::placeholders::_1));

        // ============================================
        // services / actions
        // ============================================

        fromll_client_ =
            create_client<robot_localization::srv::FromLL>(
                "/fromLL");

        nav_client_ =
            rclcpp_action::create_client<NavigateToPose>(
                this,
                "/navigate_to_pose");

        follow_client_ =
            rclcpp_action::create_client<FollowWaypoints>(
                this,
                "/follow_waypoints");

        // ============================================
        // wait
        // ============================================

        while (!fromll_client_->wait_for_service(1s))
        {
            RCLCPP_INFO(get_logger(), "Waiting /fromLL...");
        }

        while (!nav_client_->wait_for_action_server(1s))
        {
            RCLCPP_INFO(get_logger(), "Waiting /navigate_to_pose...");
        }

        while (!follow_client_->wait_for_action_server(1s))
        {
            RCLCPP_INFO(get_logger(), "Waiting /follow_waypoints...");
        }

        RCLCPP_INFO(get_logger(), "Waiting first GPS...");
    }

private:

    // ============================================================
    // variables
    // ============================================================

    std::vector<GPSWaypoint> gps_waypoints_;

    std::vector<geometry_msgs::msg::PoseStamped>
        corrected_waypoints_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr
        gps_sub_;

    rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr
        fromll_client_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr
        nav_client_;

    rclcpp_action::Client<FollowWaypoints>::SharedPtr
        follow_client_;

    bool gps_received_ = false;

    double current_lat_;
    double current_lon_;

    double first_lat_;
    double first_lon_;

    double second_lat_;
    double second_lon_;

    bool started_ = false;

    double yaw_offset_deg_ = 0.0;

    // ============================================================
    // yaml
    // ============================================================

    void loadYaml(const std::string & path)
    {
        YAML::Node config = YAML::LoadFile(path);

        for (auto wp : config["waypoints"])
        {
            GPSWaypoint p;

            p.lat = wp["latitude"].as<double>();
            p.lon = wp["longitude"].as<double>();

            gps_waypoints_.push_back(p);
        }

        RCLCPP_INFO(
            get_logger(),
            "Loaded %ld waypoints",
            gps_waypoints_.size());
    }

    // ============================================================
    // GPS callback
    // ============================================================

    void gpsCallback(
        const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        current_lat_ = msg->latitude;
        current_lon_ = msg->longitude;

        gps_received_ = true;

        // first GPS only
        if (!started_)
        {
            started_ = true;

            first_lat_ = current_lat_;
            first_lon_ = current_lon_;

            RCLCPP_INFO(
                get_logger(),
                "First GPS fixed");

            sendFirstGoal();
        }
    }

    // ============================================================
        // first goal
        // ============================================================

   void sendFirstGoal()
    {
        if (gps_waypoints_.empty())
        {
            RCLCPP_ERROR(get_logger(), "No waypoint");
            return;
        }

        auto & wp = gps_waypoints_[0];

        auto req =
            std::make_shared<
                robot_localization::srv::FromLL::Request>();

        req->ll_point.latitude = wp.lat;
        req->ll_point.longitude = wp.lon;
        req->ll_point.altitude = 0.0;

        fromll_client_->async_send_request(
            req,
            std::bind(
                &GPSWaypointInitializer::fromLLCallback,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Requested /fromLL");
    }

    void fromLLCallback(
    rclcpp::Client<
        robot_localization::srv::FromLL>::SharedFuture future)
{
    auto res = future.get();

    geometry_msgs::msg::PoseStamped pose;

    pose.header.frame_id = "map";
    pose.header.stamp = now();

    // 通常はこちら
    pose.pose.position.x = -(res->map_point.y);
    pose.pose.position.y = res->map_point.x;

    // 必要なら以前の軸変換を使う
    // pose.pose.position.x = -(res->map_point.y);
    // pose.pose.position.y =  (res->map_point.x);

    pose.pose.orientation.w = 1.0;

    NavigateToPose::Goal goal;
    goal.pose = pose;

    auto options =
        rclcpp_action::Client<NavigateToPose>
            ::SendGoalOptions();

    options.result_callback =
        std::bind(
            &GPSWaypointInitializer::firstGoalResultCallback,
            this,
            std::placeholders::_1);

    nav_client_->async_send_goal(goal, options);

    RCLCPP_INFO(
        get_logger(),
        "Sent first waypoint");
}

    // ============================================================
    // first goal result
    // ============================================================

    void firstGoalResultCallback(
        const GoalHandleNavigateToPose::WrappedResult & result)
    {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_ERROR(
                get_logger(),
                "First waypoint failed");
            return;
        }

        // ============================================
        // save arrived GPS
        // ============================================

        second_lat_ = current_lat_;
        second_lon_ = current_lon_;

        RCLCPP_INFO(
            get_logger(),
            "First waypoint arrived");

        estimateYawOffset();

        convertAllWaypoints();

        sendFollowWaypoints();
    }

    // ============================================================
    // yaw estimation
    // ============================================================

    void estimateYawOffset()
    {
        const GeographicLib::Geodesic & geod =
            GeographicLib::Geodesic::WGS84();

        // --------------------------------------------
        // ideal direction
        // --------------------------------------------

        double dist1;
        double ideal_azimuth_deg;
        double az2;

        geod.Inverse(
            first_lat_,
            first_lon_,
            gps_waypoints_[0].lat,
            gps_waypoints_[0].lon,
            dist1,
            ideal_azimuth_deg,
            az2);

        // --------------------------------------------
        // actual moved direction
        // --------------------------------------------

        double dist2;
        double actual_azimuth_deg;

        geod.Inverse(
            first_lat_,
            first_lon_,
            second_lat_,
            second_lon_,
            dist2,
            actual_azimuth_deg,
            az2);

        yaw_offset_deg_ =
            actual_azimuth_deg -
            ideal_azimuth_deg;

        RCLCPP_INFO(
            get_logger(),
            "Ideal azimuth : %.3f",
            ideal_azimuth_deg);

        RCLCPP_INFO(
            get_logger(),
            "Actual azimuth: %.3f",
            actual_azimuth_deg);

        RCLCPP_INFO(
            get_logger(),
            "Yaw offset    : %.3f deg",
            yaw_offset_deg_);
    }

    // ============================================================
    // convert all waypoints
    // ============================================================

    void convertAllWaypoints()
    {
        corrected_waypoints_.clear();

        for (size_t i = 1; i < gps_waypoints_.size(); i++)
        {
            auto & wp = gps_waypoints_[i];

            // =========================================
            // GPS relative position
            // =========================================

            const GeographicLib::Geodesic & geod =
                GeographicLib::Geodesic::WGS84();

            double distance;
            double azimuth_deg;
            double az2;

            geod.Inverse(
                second_lat_,
                second_lon_,
                wp.lat,
                wp.lon,
                distance,
                azimuth_deg,
                az2);

            // =========================================
            // yaw correction
            // =========================================

            double corrected_deg =
                azimuth_deg -
                yaw_offset_deg_;

            double yaw_rad =
                corrected_deg * M_PI / 180.0;

            // =========================================
            // robot/map coordinates
            // =========================================

            double x =
                distance * sin(yaw_rad);

            double y =
                distance * cos(yaw_rad);

            geometry_msgs::msg::PoseStamped pose;

            pose.header.frame_id = "map";
            pose.header.stamp = now();

            pose.pose.position.x = x;
            pose.pose.position.y = y;

            pose.pose.orientation.w = 1.0;

            corrected_waypoints_.push_back(pose);

            RCLCPP_INFO(
                get_logger(),
                "WP[%ld] x=%.2f y=%.2f",
                i,
                x,
                y);
        }
    }

    // ============================================================
    // send follow_waypoints
    // ============================================================

    void sendFollowWaypoints()
    {
        FollowWaypoints::Goal goal;

        goal.poses = corrected_waypoints_;

        auto options =
            rclcpp_action::Client<FollowWaypoints>
                ::SendGoalOptions();

        options.feedback_callback =
            [](auto, auto feedback)
            {
                RCLCPP_INFO(
                    rclcpp::get_logger("follow"),
                    "Current WP: %d",
                    feedback->current_waypoint);
            };

        follow_client_->async_send_goal(
            goal,
            options);

        RCLCPP_INFO(
            get_logger(),
            "Sent follow_waypoints");
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    if (argc < 2)
    {
        std::cout
            << "usage: ros2 run pkg node waypoints.yaml"
            << std::endl;

        return 1;
    }

    auto node =
        std::make_shared<GPSWaypointInitializer>(
            argv[1]);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}