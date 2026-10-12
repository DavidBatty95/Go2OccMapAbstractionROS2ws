import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from go2_occmap.sim_node import Simulator
from go2_occmap.explore_node import Explorer


def test_live_dds_odometry_angular_velocity_and_joint_contract(tmp_path):
    rclpy.init(args=['--ros-args', '-p', f'output:={tmp_path}'])
    simulator = Simulator()
    observer = rclpy.create_node('contract_test_observer')
    received = {}
    observer.create_subscription(Odometry, '/odom', lambda m: received.update(odom=m), 10)
    observer.create_subscription(JointState, '/joint_states', lambda m: received.update(joints=m), 10)
    try:
        # A known free-base yaw rate must appear in the actual DDS odometry message.
        simulator.sim.data.qvel[5] = .4
        deadline = time.monotonic()+5
        while len(received) < 2 and time.monotonic() < deadline:
            simulator.publish_state()
            rclpy.spin_once(observer, timeout_sec=.05)
        assert len(received) == 2
        assert received['odom'].header.frame_id == 'odom'
        assert .39 < received['odom'].twist.twist.angular.z < .41
        assert len(received['joints'].name) == len(received['joints'].position) == 12
    finally:
        observer.destroy_node()
        simulator.destroy_node()
        rclpy.shutdown()


def test_unsupported_motion_wait_has_a_bounded_failure_exit(tmp_path):
    rclpy.init(args=['--ros-args', '-p', f'output:={tmp_path}'])
    explorer = Explorer()
    try:
        explorer.pose = np.array([-2., 0., 0.])
        explorer.occupancy = np.full((80, 100), -1)
        explorer.last_odom = explorer.last_map = explorer.last_sensor = time.monotonic()
        explorer.control()
        assert explorer.state == 'waiting_for_support'
        explorer.wait_started -= 5.
        explorer.control()
        assert explorer.state == 'failed'
        assert explorer.trace[-1]['reason'] == 'unsupported_motion_timeout'
        explorer.control()
        assert explorer.state == 'failed'
    finally:
        explorer.destroy_node()
        rclpy.shutdown()
