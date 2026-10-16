
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import LaserScan
from astar_navigation.lidar_safety import front_distance
import time
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import SetBool
import numpy as np
import heapq
import math


class AStarNavigation(Node):
    def __init__(self):
        super().__init__('astar_navigation')

        self.map_data = None
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        self.declare_parameter('stop_distance', 0.4)
        self.declare_parameter('release_distance', 0.5)
        self.declare_parameter('front_half_angle_deg', 20.0)
        self.declare_parameter('scan_timeout', 1.0)
        self.stop_distance = self.get_parameter('stop_distance').value
        self.release_distance = self.get_parameter('release_distance').value
        self.front_half_angle = math.radians(self.get_parameter('front_half_angle_deg').value)
        self.scan_timeout = self.get_parameter('scan_timeout').value
        if not (0 < self.stop_distance < self.release_distance
                and 0 < self.front_half_angle <= math.pi
                and self.scan_timeout > 0
                and all(math.isfinite(v) for v in (
                    self.stop_distance, self.release_distance,
                    self.front_half_angle, self.scan_timeout))):
            raise ValueError('Invalid laser safety distances, angle or timeout')
        self.last_scan_time = None
        self.front_distance = None
        self.emergency_stopped = False
        self.path = []
        self.path_index = 0
        self.goal_world = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos
        )
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        self.path_pub = self.create_publisher(
            Path, '/astar_path', 10
        )

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.create_service(
            SetBool, '/emergency_stop', self.emergency_stop_callback
        )
        self.safety_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self.control_loop, clock=self.safety_clock)

        self.get_logger().info('A* Navigation node started')

    def scan_is_fresh(self):
        return (self.last_scan_time is not None
                and time.monotonic() - self.last_scan_time <= self.scan_timeout
                and self.front_distance is not None)

    def engage_stop(self, reason):
        was_stopped = self.emergency_stopped
        self.emergency_stopped = True
        self.path = []
        self.path_index = 0
        self.goal_world = None
        self.stop_robot()
        self.publish_path()
        if not was_stopped:
            self.get_logger().warning(reason)

    def scan_callback(self, msg):
        self.front_distance = front_distance(msg, self.front_half_angle)
        self.last_scan_time = time.monotonic()
        if self.front_distance is not None and self.front_distance <= self.stop_distance:
            self.engage_stop(f'Laser emergency stop: {self.front_distance:.2f} m ahead')
        elif self.front_distance is None and self.path:
            self.engage_stop('Laser emergency stop: unusable front scan')

    def emergency_stop_callback(self, request, response):
        if request.data:
            self.engage_stop('Manual emergency stop engaged')
            response.success = True
            response.message = 'Emergency stop engaged'
        elif not self.scan_is_fresh():
            self.engage_stop('Release denied: fresh usable scan required')
            response.success = False
            response.message = 'Release denied: fresh usable scan required'
        elif self.front_distance < self.release_distance:
            self.engage_stop('Release denied: obstacle still too close')
            response.success = False
            response.message = f'Release denied: front clearance must be at least {self.release_distance:.2f} m'
        else:
            self.emergency_stopped = False
            self.path = []
            self.path_index = 0
            self.goal_world = None
            self.stop_robot()
            response.success = True
            response.message = 'Emergency stop released. Set a new goal to drive.'
        self.get_logger().info(response.message)
        return response

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        self.map_data = np.array(
            msg.data, dtype=np.int8
        ).reshape((msg.info.height, msg.info.width))

    def world_to_grid(self, x, y):
        col = int((x - self.map_origin_x) / self.map_resolution)
        row = int((y - self.map_origin_y) / self.map_resolution)
        return row, col

    def grid_to_world(self, row, col):
        x = self.map_origin_x + (col + 0.5) * self.map_resolution
        y = self.map_origin_y + (row + 0.5) * self.map_resolution
        return x, y

    def goal_callback(self, msg):
        if not self.scan_is_fresh():
            self.stop_robot()
            self.get_logger().warning('Goal rejected: fresh usable scan required')
            return
        if self.emergency_stopped:
            self.stop_robot()
            self.get_logger().warning('Goal rejected: emergency stop engaged')
            return
        if self.map_data is None:
            self.get_logger().warning('Map is not available')
            return

        self.goal_world = (
            msg.pose.position.x,
            msg.pose.position.y
        )
        goal = self.world_to_grid(*self.goal_world)

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            start = self.world_to_grid(x, y)

            path = self.a_star(start, goal)

            if path:
                self.path = path
                self.path_index = 1 if len(path) > 1 else 0
                self.publish_path()

                self.get_logger().info(
                    f'A* path found: {len(path)} cells'
                )
            else:
                self.path = []
                self.publish_path()
                self.stop_robot()
                self.get_logger().warning('No path found')

        except Exception as e:
            self.get_logger().warning(f'Navigation error: {e}')

    def a_star(self, start, goal):
        height, width = self.map_data.shape

        for row, col in (start, goal):
            if not (0 <= row < height and 0 <= col < width):
                self.get_logger().warning('Position outside map')
                return None

        def is_walkable(row, col):
            value = int(self.map_data[row, col])
            return 0 <= value < 65

        if not is_walkable(*start) or not is_walkable(*goal):
            self.get_logger().warning(
                'Start or goal cell is unknown/occupied'
            )
            return None

        open_list = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        closed_set = set()

        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1)
        ]

        while open_list:
            _, current = heapq.heappop(open_list)

            if current in closed_set:
                continue

            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            closed_set.add(current)

            for dr, dc in directions:
                neighbor = (
                    current[0] + dr,
                    current[1] + dc
                )
                row, col = neighbor

                if not (0 <= row < height and 0 <= col < width):
                    continue

                if not is_walkable(row, col):
                    continue

                tentative_g = g_score[current] + 1

                if tentative_g < g_score.get(
                    neighbor, float('inf')
                ):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    h = abs(goal[0] - row) + abs(goal[1] - col)
                    heapq.heappush(
                        open_list,
                        (tentative_g + h, neighbor)
                    )

        return None

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for row, col in self.path:
            x, y = self.grid_to_world(row, col)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def control_loop(self):
        if self.emergency_stopped:
            self.stop_robot()
            return
        if not self.path:
            return
        if not self.scan_is_fresh():
            self.engage_stop('Laser emergency stop: scan missing or stale')
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
        except Exception:
            self.stop_robot()
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        cmd = Twist()

        row, col = self.path[self.path_index]
        target_x, target_y = self.grid_to_world(row, col)

        distance = math.hypot(target_x - x, target_y - y)
        angle_to_target = math.atan2(
            target_y - y, target_x - x
        )
        angle_error = math.atan2(
            math.sin(angle_to_target - yaw),
            math.cos(angle_to_target - yaw)
        )

        if distance < 0.04:
            if self.path_index < len(self.path) - 1:
                self.path_index += 1
            else:
                target_x, target_y = self.goal_world
                distance = math.hypot(
                    target_x - x, target_y - y
                )

                if distance < 0.08:
                    self.stop_robot()
                    self.path = []
                    self.publish_path()
                    self.get_logger().info('Goal reached')
                    return

                angle_to_target = math.atan2(
                    target_y - y, target_x - x
                )
                angle_error = math.atan2(
                    math.sin(angle_to_target - yaw),
                    math.cos(angle_to_target - yaw)
                )

        if abs(angle_error) > 0.20:
            cmd.angular.z = max(
                -0.8, min(0.8, 1.5 * angle_error)
            )
        else:
            cmd.linear.x = min(0.08, 0.8 * distance)
            cmd.angular.z = max(
                -0.5, min(0.5, angle_error)
            )

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = AStarNavigation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
