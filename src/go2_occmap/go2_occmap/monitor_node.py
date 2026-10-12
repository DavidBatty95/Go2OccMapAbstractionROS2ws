"""Independent DDS subscriber validates the live simulation interface contract."""
import json
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float64MultiArray, String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray


class Monitor(Node):
    def __init__(self):
        super().__init__('interface_monitor')
        self.declare_parameter('output', '/workspace/results/latest')
        self.counts, self.invalid = {}, {}
        topics = [('/clock', Clock), ('/odom', Odometry), ('/scan', LaserScan), ('/imu', Imu),
                  ('/joint_states', JointState), ('/cmd_vel', Twist), ('/map', OccupancyGrid),
                  ('/plan', NavPath), ('/abstraction/graph', MarkerArray),
                  ('/radiation/measurement', Float64MultiArray), ('/mission/status', String), ('/tf', TFMessage)]
        for topic, kind in topics:
            self.counts[topic], self.invalid[topic] = 0, 0
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic == '/map' else 30
            self.create_subscription(kind, topic, lambda message, key=topic: self.receive(key, message), qos)

    def receive(self, topic, message):
        self.counts[topic] += 1
        valid = True
        if topic == '/joint_states':
            valid = len(message.name) == len(set(message.name)) == len(message.position) == 12 and np.isfinite(message.position).all()
        elif topic == '/scan':
            valid = len(message.ranges) == 180 and not np.isnan(message.ranges).any()
        elif topic == '/map':
            valid = len(message.data) == 8000 and set(message.data).issubset({-1, 0, 100})
        elif topic == '/radiation/measurement':
            valid = len(message.data) == 5 and np.isfinite(message.data).all()
        elif topic == '/cmd_vel':
            valid = np.isfinite([message.linear.x, message.linear.y, message.angular.z]).all()
        elif topic == '/mission/status':
            try:
                valid = 'state' in json.loads(message.data)
            except ValueError:
                valid = False
        self.invalid[topic] += int(not valid)

    def save(self):
        path = Path(self.get_parameter('output').value)
        path.mkdir(parents=True, exist_ok=True)
        (path/'ros_interfaces.json').write_text(json.dumps({'messages': self.counts, 'invalid': self.invalid}, indent=2)+'\n')


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
