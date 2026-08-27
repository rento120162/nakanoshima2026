#! /usr/bin/env python3

import yaml

import rclpy

from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped

from robot_localization.srv import FromLL

from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult,
)


class GPSWaypointFollower(Node):

    def __init__(self):

        super().__init__('gps_waypoint_follower')

        #
        # Nav2
        #
        self.navigator = BasicNavigator()

        #
        # fromLL service
        #
        self.fromll_client = self.create_client(
            FromLL,
            '/fromLL'
        )

        #
        # attribute publisher
        #
        self.attribute_pub = self.create_publisher(
            String,
            '/waypoint_attribute',
            10
        )

        self.get_logger().info(
            'Waiting for /fromLL service...'
        )

        while not self.fromll_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                '/fromLL not available...'
            )

        self.get_logger().info(
            '/fromLL connected'
        )

    #
    # GPS -> map
    #
    def gps_to_map(self, lat, lon, alt=0.0):

        req = FromLL.Request()

        req.ll_point.latitude = float(lat)
        req.ll_point.longitude = float(lon)
        req.ll_point.altitude = float(alt)

        future = self.fromll_client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        result = future.result()

        if result is None:

            self.get_logger().error(
                'fromLL failed'
            )

            return None

        return result.map_point

    #
    # pose生成
    #
    def make_pose(self, x, y):

        pose = PoseStamped()

        pose.header.frame_id = 'map'

        pose.header.stamp = (
            self.navigator.get_clock()
            .now()
            .to_msg()
        )

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        #
        # yaw無視
        #
        pose.pose.orientation.w = 1.0

        return pose

    #
    # attribute publish
    #
    def publish_attribute(self, attribute):

        msg = String()

        msg.data = attribute

        self.attribute_pub.publish(msg)

        self.get_logger().info(
            f'Published attribute: {attribute}'
        )

    #
    # YAML load
    #
    def load_waypoints(self, yaml_file):

        with open(yaml_file, 'r') as f:

            data = yaml.safe_load(f)

        return data['waypoints']

    #
    # main
    #
    def run(self, yaml_file):

        #
        # initial pose
        #
        initial_pose = PoseStamped()

        initial_pose.header.frame_id = 'map'

        initial_pose.header.stamp = (
            self.navigator.get_clock()
            .now()
            .to_msg()
        )

        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.w = 1.0

        self.navigator.setInitialPose(
            initial_pose
        )

        #
        # wait nav2
        #
        self.navigator.waitUntilNav2Active()

        #
        # load yaml
        #
        waypoints = self.load_waypoints(
            yaml_file
        )

        #
        # one-by-one navigation
        #
        for i, wp in enumerate(waypoints):

            lat = wp['latitude']
            lon = wp['longitude']

            #
            # optional attribute
            #
            attribute = wp.get(
                'attribute',
                ''
            )

            self.get_logger().info(
                f'Waypoint {i}'
            )

            self.get_logger().info(
                f'GPS: {lat}, {lon}'
            )

            #
            # GPS -> map
            #
            map_point = self.gps_to_map(
                lat,
                lon
            )

            if map_point is None:

                continue

            self.get_logger().info(
                f'Map XY: '
                f'{map_point.x}, '
                f'{map_point.y}'
            )

            #
            # pose
            #
            goal_pose = self.make_pose(
                map_point.x,
                map_point.y
            )

            #
            # send goal
            #
            self.navigator.goToPose(
                goal_pose
            )

            #
            # wait result
            #
            while not self.navigator.isTaskComplete():

                feedback = (
                    self.navigator.getFeedback()
                )

                if feedback:

                    self.get_logger().info(
                        f'Distance remaining: '
                        f'{feedback.distance_remaining:.2f}'
                    )

                rclpy.spin_once(
                    self,
                    timeout_sec=0.5
                )

            #
            # result
            #
            result = (
                self.navigator.getResult()
            )

            if result == TaskResult.SUCCEEDED:

                self.get_logger().info(
                    'Waypoint reached'
                )

                #
                # publish attribute
                #
                if attribute != '':

                    self.publish_attribute(
                        attribute
                    )

            elif result == TaskResult.FAILED:

                self.get_logger().error(
                    'Goal failed'
                )

            elif result == TaskResult.CANCELED:

                self.get_logger().warn(
                    'Goal canceled'
                )

        #
        # shutdown
        #
        self.navigator.lifecycleShutdown()


def main():

    rclpy.init()

    import sys

    if len(sys.argv) < 2:

        print(
            'usage: '
            'gps_waypoint_follower.py '
            'waypoints.yaml'
        )

        return

    yaml_file = sys.argv[1]

    node = GPSWaypointFollower()

    node.run(yaml_file)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()