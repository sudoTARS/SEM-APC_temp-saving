import rclpy
from rclpy.node import Node
import carla
import math
from scipy.spatial import distance
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from visualization_msgs.msg import MarkerArray, Marker
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np

class CarlaMapPublisher(Node):
    def __init__(self):
        super().__init__('Carla_navigation_hmi')

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()

        self.publisher_ = self.create_publisher(MarkerArray, 'carla_road_network', 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, "/carla/hero/goal_pose", 10)
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.control_speed_publisher = self.create_publisher(Float64, '/carla/hero/speed_command', qos_profile)

        self.init_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.initial_pose_callback,
            qos_profile=10)

        self.goal_pose_subscriber = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_pose_callback,
            qos_profile=10)

        self.odom_subscriber = self.create_subscription(
            Odometry,
            "/carla/ego_vehicle/odometry",
            self.odom_callback,
            qos_profile=10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.waypoints = self.map.generate_waypoints(0.7)
        self.markers = self.create_markers(self.waypoints)
        
        self.current_pose = None
        self.current_goal_idx = 0

        self.start_position = (280.363739, -129.306351, 0.101746)
        raw_goals = [
            [334.949799, -161.106171, 0.001736],
            [339.100037, -258.568939, 0.001679],
            [396.295319, -183.195740, 0.001678],
            [267.657074, -1.983160, 0.001678],
            [153.868896, -26.115866, 0.001678],
            [290.515564, -56.175072, 0.001677],
            [92.325722, -86.063644, 0.001677],
            [88.384346, -287.468567, 0.001728],
            [177.594101, -326.386902, 0.001677],
            [-1.646942, -197.501282, 0.001555],
            [59.701321, -1.970804, 0.001467],
            [122.100121, -55.142044, 0.001596],
            [161.030975, -129.313187, 0.001679],
            [184.758713, -199.424271, 0.001680]
        ]
        
        self.goal_waypoints = self.get_optimized_path(self.start_position, raw_goals)

        # Set initial pose
        self.publish_initial_pose()

        # Start with the first goal
        self.publish_goal_pose(self.goal_waypoints[self.current_goal_idx])

    def get_optimized_path(self, start, goals):
        remaining = goals.copy()
        path = []
        current = start
        while remaining:
            nearest = min(remaining, key=lambda point: distance.euclidean(current[:2], point[:2]))
            path.append(nearest)
            current = nearest
            remaining.remove(nearest)
        return path

    def initial_pose_callback(self, msg):
        self.get_logger().info(f"Initial Pose: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y})")

    def goal_pose_callback(self, msg):
        self.get_logger().info(f"Goal Reached: ({msg.pose.position.x}, {msg.pose.position.y})")
        speed_msg = Float64()
        speed_msg.data = 48.0  # km/h
        self.control_speed_publisher.publish(speed_msg)
        self.goal_pose_publisher.publish(msg)
        self.get_logger().info("Published goal")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.check_goal_reached()

    def check_goal_reached(self):
        if self.current_pose is None or self.current_goal_idx >= len(self.goal_waypoints):
            return

        goal = self.goal_waypoints[self.current_goal_idx]
        dx = self.current_pose.position.x - goal[0]
        dy = self.current_pose.position.y - goal[1]
        dist = math.hypot(dx, dy)

        if dist < 2.0:
            self.get_logger().info(f"Reached waypoint {self.current_goal_idx}")
            self.current_goal_idx += 1
            if self.current_goal_idx < len(self.goal_waypoints):
                self.publish_goal_pose(self.goal_waypoints[self.current_goal_idx])
            else:
                self.get_logger().info("All goal waypoints reached.")

    def publish_goal_pose(self, goal):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = goal[2]
        msg.pose.orientation.w = 1.0
        self.goal_pose_publisher.publish(msg)
        self.get_logger().info(f"Published new goal: {goal[0]}, {goal[1]}")

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.start_position[0]
        msg.pose.pose.position.y = self.start_position[1]
        msg.pose.pose.position.z = self.start_position[2]
        msg.pose.pose.orientation.w = 1.0
        self.initial_pose_callback(msg)

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
