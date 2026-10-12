"""Build observed occupancy from timestamp-matched ideal pose and laser scans."""
from collections import deque
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

from .mapping import ObservedGrid, ORIGIN, RESOLUTION, SHAPE


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class Mapper(Node):
    def __init__(self):
        super().__init__('observed_grid_mapper')
        self.declare_parameter('output', '/workspace/results/latest')
        self.grid = ObservedGrid()
        self.poses, self.scans = deque(maxlen=300), deque(maxlen=30)
        self.updates = self.unmatched = 0
        self.snapshots, self.times = [], []
        self.last_snapshot = -10.
        self.create_subscription(Odometry, '/odom', self.odom, 100)
        self.create_subscription(LaserScan, '/scan', self.scans.append, 30)
        self.publisher = self.create_publisher(OccupancyGrid, '/map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.static_tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
        transform.transform.rotation.w = 1.
        self.static_tf.sendTransform(transform)
        self.create_timer(.1, self.process)

    def odom(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        self.poses.append((stamp_seconds(message.header.stamp), [p.x, p.y, 2*math.atan2(q.z, q.w)]))

    def process(self):
        if not self.poses:
            return
        last = None
        while self.scans:
            scan = self.scans[0]
            stamp = stamp_seconds(scan.header.stamp)
            nearest = min(self.poses, key=lambda p: abs(p[0]-stamp))
            if abs(nearest[0]-stamp) > .025:
                if stamp > self.poses[-1][0]:
                    break
                self.scans.popleft()
                self.unmatched += 1
                continue
            self.scans.popleft()
            self.grid.integrate(nearest[1], scan.ranges, scan.angle_min, scan.angle_increment, scan.range_max)
            self.updates += 1
            last = scan
        if last is None:
            return
        occupancy = self.grid.occupancy()
        message = OccupancyGrid()
        message.header.stamp, message.header.frame_id = last.header.stamp, 'map'
        message.info.resolution, message.info.height, message.info.width = RESOLUTION, SHAPE[0], SHAPE[1]
        message.info.origin.position.x, message.info.origin.position.y = map(float, ORIGIN)
        message.info.origin.orientation.w = 1.
        message.data = occupancy.reshape(-1).astype(int).tolist()
        self.publisher.publish(message)
        stamp = stamp_seconds(last.header.stamp)
        if stamp-self.last_snapshot >= 2.:
            self.snapshots.append(occupancy)
            self.times.append(stamp)
            self.last_snapshot = stamp

    def save(self):
        output = Path(self.get_parameter('output').value)
        output.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(output/'mapping.npz', grid=self.grid.occupancy(), snapshots=np.asarray(self.snapshots),
                            time=self.times, origin=ORIGIN, resolution=RESOLUTION)
        (output/'mapping.json').write_text(json.dumps(dict(scan_updates=self.updates, unmatched_scans=self.unmatched,
            observed_free_cells=int(np.sum(self.grid.occupancy() == 0)),
            occupied_cells=int(np.sum(self.grid.occupancy() == 100))), indent=2)+'\n')


def main():
    rclpy.init()
    node = Mapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
