"""Exercise laser filtering and stop/release behavior over ROS2 transport."""
import math
import time
import unittest
from types import SimpleNamespace

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist, PoseStamped
from astar_navigation.astar_node import AStarNavigation
from astar_navigation.lidar_safety import front_distance


class LaserSafetyTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.nav = AStarNavigation()
        self.probe = Node('laser_safety_probe')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.nav)
        self.executor.add_node(self.probe)
        self.publisher = self.probe.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.client = self.probe.create_client(SetBool, '/emergency_stop')
        self.velocities = []
        self.probe.create_subscription(Twist, '/cmd_vel', self.velocities.append, 10)
        self.assertTrue(self.client.wait_for_service(timeout_sec=5))
        self.spin(.3)

    def tearDown(self):
        self.executor.shutdown()
        self.nav.destroy_node()
        self.probe.destroy_node()
        rclpy.shutdown()

    def spin(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)

    def scan(self, distance, index=0):
        msg = LaserScan()
        msg.angle_min = 0.0
        msg.angle_increment = math.pi / 180
        msg.range_min = .12
        msg.range_max = 3.5
        msg.ranges = [math.inf] * 360
        msg.ranges[index] = distance
        for _ in range(3):
            self.publisher.publish(msg)
            self.spin(.05)

    def call(self, stop):
        future = self.client.call_async(SetBool.Request(data=stop))
        end = time.monotonic() + 3
        while not future.done() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)
        self.assertTrue(future.done())
        return future.result()

    def test_obstacle_latches_and_requires_clearance_and_new_goal(self):
        self.nav.path = [(0, 0)]
        self.scan(.3)
        self.assertTrue(self.nav.emergency_stopped)
        self.assertEqual(self.nav.path, [])
        self.assertFalse(self.call(False).success)
        self.scan(.45)
        self.assertFalse(self.call(False).success)
        self.scan(.8)
        self.assertTrue(self.nav.emergency_stopped)
        self.nav.goal_callback(PoseStamped())
        self.assertEqual(self.nav.path, [])
        self.spin(.15)
        self.assertTrue(self.velocities)
        self.assertTrue(all(v.linear.x == 0.0 and v.angular.z == 0.0 for v in self.velocities))
        self.assertTrue(self.call(False).success)
        self.assertFalse(self.nav.emergency_stopped)
        self.assertEqual(self.nav.path, [])

    def test_rear_obstacle_is_ignored_and_manual_stop_works(self):
        self.scan(.2, index=180)
        self.assertFalse(self.nav.emergency_stopped)
        self.assertTrue(self.call(True).success)
        self.assertTrue(self.call(False).success)

    def test_stale_scan_blocks_release_and_stops_active_route(self):
        self.scan(.8)
        self.nav.last_scan_time = time.monotonic() - 2
        self.nav.path = [(0, 0)]
        self.nav.control_loop()
        self.assertTrue(self.nav.emergency_stopped)
        self.assertFalse(self.call(False).success)

    def test_invalid_scan_blocks_release(self):
        msg = LaserScan()
        self.nav.scan_callback(msg)
        self.assertFalse(self.call(False).success)

    def test_filter_handles_angle_wrap_nan_and_infinity(self):
        scan = SimpleNamespace(angle_min=-math.pi, angle_increment=math.pi/180,
                               range_min=.12, range_max=3.5, ranges=[math.inf]*360)
        self.assertEqual(front_distance(scan, math.radians(20)), math.inf)
        scan.ranges[180] = .3
        self.assertAlmostEqual(front_distance(scan, math.radians(20)), .3)
        scan.ranges = [math.nan]*360
        self.assertIsNone(front_distance(scan, math.radians(20)))
        scan.angle_increment = 0.
        self.assertIsNone(front_distance(scan, math.radians(20)))


if __name__ == '__main__':
    unittest.main()
