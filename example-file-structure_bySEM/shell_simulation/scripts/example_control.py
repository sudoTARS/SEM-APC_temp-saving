#!/usr/bin/env python3

import os
import sys

# Set CARLA Python API path
carla_root = os.path.expanduser('~/carla_simulator')
carla_egg_path = os.path.join(carla_root, 'PythonAPI', 'carla')

if carla_egg_path not in sys.path:
    sys.path.append(carla_egg_path)

try:
    import carla
except ModuleNotFoundError as e:
    raise ImportError(f"CARLA Python API not found at {carla_egg_path}. Ensure CARLA is installed.") from e


import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
import carla
import math
from scipy.spatial import distance

class CarlaMapPublisher(Node):
    def __init__(self):
        super().__init__('Carla_navigation_hmi')

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()

        self.waypoints = self.map.generate_waypoints(0.7)
        self.publisher_ = self.create_publisher(MarkerArray, 'carla_road_network', 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, "/carla/ego_vehicle/goal", 10)
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.control_speed_publisher = self.create_publisher(Float64, '/carla/ego_vehicle/speed_command', qos_profile)
        self.odom_subscriber = self.create_subscription(Odometry, "/carla/ego_vehicle/odometry", self.odom_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.markers = self.create_markers(self.waypoints)

        self.start_position = (280.363739, -129.306351, 0.101746)
        self.goal_waypoints = [
            [161.030975, -129.313187, 0.001679],
            [184.758713, -199.424271, 0.001680],
            [334.949799, -161.106171, 0.001736],
            [290.515564, -56.175072, 0.001677],
            [122.100121, -55.142044, 0.001596],
            [339.100037, -258.568939, 0.001679],
            [177.594101, -326.386902, 0.001677],
            [88.384346, -287.468567, 0.001728],
            [59.701321, -1.970804, 0.001467],
            [-1.646942, -197.501282, 0.001555],
            [396.295319, -183.195740, 0.001678],
            [267.657074, -1.983160, 0.001678],
            [153.868896, -26.115866, 0.001678],
            [92.325722, -86.063644, 0.001677]  
        ]

        self.current_goal_idx = 0
        self.goal_active = False
        self.goal_departed = False
        self.current_pose = None

        self.publish_initial_pose()
        self.publish_next_goal()

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.start_position[0]
        msg.pose.pose.position.y = self.start_position[1]
        msg.pose.pose.position.z = self.start_position[2]
        msg.pose.pose.orientation.w = 1.0
        self.get_logger().info("Published initial pose")

    def publish_next_goal(self):
        if self.current_goal_idx >= len(self.goal_waypoints):
            self.get_logger().info("All waypoints reached.")
            return

        goal = self.goal_waypoints[self.current_goal_idx]
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = goal[2]
        msg.pose.orientation.w = 1.0

        speed_msg = Float64()
        speed_msg.data = 32.0

        self.goal_pose_publisher.publish(msg)
        self.control_speed_publisher.publish(speed_msg)
        self.get_logger().info(f"Published goal {self.current_goal_idx + 1}: {goal}")

        self.goal_active = True
        self.goal_departed = False

    def odom_callback(self, msg):
        if not self.goal_active:
            return

        self.current_pose = msg.pose.pose
        goal = self.goal_waypoints[self.current_goal_idx]
        dx = self.current_pose.position.x - goal[0]
        dy = self.current_pose.position.y - goal[1]
        dist = math.hypot(dx, dy)

        if not self.goal_departed and dist > 0.5:
            self.goal_departed = True

        if self.goal_departed and dist < 5.0:
            self.get_logger().info(f"Reached waypoint {self.current_goal_idx + 1}")
            self.current_goal_idx += 1
            self.goal_active = False
            self.goal_departed = False
            self.publish_next_goal()

    def create_markers(self, waypoints):
        marker_array = MarkerArray()
        for idx, wp in enumerate(waypoints):
            marker = self.create_marker(wp, idx)
            marker_array.markers.append(marker)
        return marker_array

    def create_marker(self, waypoint, idx):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "carla_waypoints"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = waypoint.transform.location.x
        marker.pose.position.y = -waypoint.transform.location.y
        marker.pose.position.z = -waypoint.transform.location.z
        return marker

    def timer_callback(self):
        self.publisher_.publish(self.markers)


def main(args=None):
    rclpy.init(args=args)
    node = CarlaMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
