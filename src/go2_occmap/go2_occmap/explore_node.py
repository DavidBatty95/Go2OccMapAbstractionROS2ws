"""Observed-free graph exploration with sampled radiation risk and return reserve."""
import json
import math
from pathlib import Path as FilePath
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

from .mapping import SampledField, SparseGraph, cell, follow_path


class Explorer(Node):
    def __init__(self):
        super().__init__('radiation_graph_explorer')
        for name, value in [('duration', 120.), ('dose_budget', 2.), ('radiation_weight', 1.),
                            ('output', '/workspace/results/latest')]:
            self.declare_parameter(name, value)
        self.pose, self.home, self.occupancy = None, np.array([-2., 0.]), None
        self.field = SampledField()
        self.path, self.goal = np.empty((0, 2)), None
        self.state, self.dose = 'initializing', 0.
        self.wait_started = None
        self.last_odom = self.last_map = self.last_sensor = -math.inf
        self.last_plan = -math.inf
        self.visited = []
        self.trace, self.planning_ms = [], []
        self.max_nodes = self.max_edges = self.command_count = self.samples = 0
        self.output = FilePath(self.get_parameter('output').value)
        self.output.mkdir(parents=True, exist_ok=True)
        self.create_subscription(Odometry, '/odom', self.odom, 30)
        self.create_subscription(OccupancyGrid, '/map', self.map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Float64MultiArray, '/radiation/measurement', self.measure, 30)
        self.velocity = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status = self.create_publisher(String, '/mission/status', 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.graph_pub = self.create_publisher(MarkerArray, '/abstraction/graph', 10)
        self.create_timer(.1, self.control)

    def odom(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        self.pose = np.array([p.x, p.y, 2*math.atan2(q.z, q.w)])
        self.last_odom = time.monotonic()

    def map(self, message):
        if message.info.height != 80 or message.info.width != 100 or abs(message.info.resolution-.1) > 1e-6:
            raise ValueError('Unexpected occupancy geometry')
        self.occupancy = np.asarray(message.data).reshape(message.info.height, message.info.width)
        self.last_map = time.monotonic()

    def measure(self, message):
        if len(message.data) != 5 or not np.isfinite(message.data).all():
            return
        _, x, y, rate, dose = message.data
        self.field.add([x, y], rate)
        self.dose, self.last_sensor = dose, time.monotonic()
        self.samples += 1

    def plan(self, now):
        started = time.monotonic()
        graph = SparseGraph(self.occupancy)
        self.max_nodes = max(self.max_nodes, len(graph.xy))
        self.max_edges = max(self.max_edges, sum(map(len, graph.edges))//2)
        anchor, home = graph.anchor(self.pose[:2], recovery=True), graph.anchor(self.home)
        self.path = np.empty((0, 2))
        if anchor is None or home is None:
            self.state = 'waiting_for_support' if self.state != 'returning' else self.state
            self.record_wait(now, graph, 'no_checked_graph_attachment')
            return
        _, upper = self.field.predict(graph.xy)
        costs, lengths, parent = graph.search(anchor, upper, self.get_parameter('radiation_weight').value)
        _, home_lengths, home_parent = graph.search(home, upper, self.get_parameter('radiation_weight').value)
        outward_dose = graph.exposure(parent, lengths, anchor, upper)
        return_dose = graph.exposure(home_parent, home_lengths, home, upper)
        budget = self.get_parameter('dose_budget').value
        duration = self.get_parameter('duration').value
        if not np.isfinite(lengths[home]):
            self.state = 'no_return_route'
            self.record_wait(now, graph, 'home_disconnected')
            return
        return_time = lengths[home]/.12+8.
        if self.dose >= .8*budget or now+return_time >= duration or self.state == 'returning':
            self.state, self.goal = 'returning', self.home.copy()
            goal_id = home
        else:
            self.state = 'exploring'
            info = graph.information[graph.cells[:, 0], graph.cells[:, 1]]
            if self.visited:
                distance_to_visit = np.linalg.norm(graph.xy[:, None]-np.asarray(self.visited)[None, :], axis=-1).min(axis=1)
            else:
                distance_to_visit = np.full(len(graph.xy), np.inf)
            score = info/(1+costs)
            candidate = (lengths > .6) & np.isfinite(costs) & np.isfinite(home_lengths) & (distance_to_visit > .6)
            # Route quadrature + dwell allowance + reserve; still an approximate model.
            candidate &= self.dose+outward_dose+return_dose+upper*5/3600+.15*budget < budget
            candidate &= now+(lengths+home_lengths)/.12+8 < duration
            score[~candidate] = -1
            goal_id = None
            if self.goal is not None and np.linalg.norm(self.goal-self.pose[:2]) > .35:
                old = graph.anchor(self.goal)
                if old is not None and candidate[old]:
                    goal_id = old
            if goal_id is None:
                if float(np.max(score, initial=-1)) <= .003:
                    self.state, self.goal, goal_id = 'returning', self.home.copy(), home
                else:
                    goal_id = int(np.argmax(score))
                    self.goal = graph.xy[goal_id].copy()
        if self.state == 'returning' and np.linalg.norm(self.pose[:2]-self.home) < .35:
            self.state = 'returned'
        else:
            self.path = graph.path(parent, anchor, goal_id)
            self.path = np.vstack([self.pose[:2], self.path])
        self.publish_graph(graph)
        self.planning_ms.append((time.monotonic()-started)*1000)
        summary = dict(time_s=now, state=self.state, pose=self.pose.tolist(), goal=self.goal.tolist(),
            graph_nodes=len(graph.xy), graph_edges=sum(map(len, graph.edges))//2,
            observed_free_cells=int(np.sum(self.occupancy == 0)), measured_dose_usv=self.dose,
            gp_samples=len(self.field.xy), path=self.path.tolist())
        self.trace.append(summary)
        self.status.publish(String(data=json.dumps(summary)))

    def record_wait(self, now, graph, reason):
        summary = dict(time_s=now, state=self.state, reason=reason, pose=self.pose.tolist(),
            goal=None if self.goal is None else self.goal.tolist(), graph_nodes=len(graph.xy),
            graph_edges=sum(map(len, graph.edges))//2, observed_free_cells=int(np.sum(self.occupancy == 0)),
            measured_dose_usv=self.dose, gp_samples=len(self.field.xy), path=[])
        self.trace.append(summary)
        self.status.publish(String(data=json.dumps(summary)))

    def publish_graph(self, graph):
        nodes, edges = Marker(), Marker()
        for marker in (nodes, edges):
            marker.header.frame_id, marker.header.stamp = 'map', self.get_clock().now().to_msg()
            marker.pose.orientation.w = 1.
            marker.action = Marker.ADD
            marker.ns = 'abstraction'
        nodes.id, nodes.type = 0, Marker.SPHERE_LIST
        nodes.scale.x = nodes.scale.y = nodes.scale.z = .08
        nodes.color.r, nodes.color.g, nodes.color.b, nodes.color.a = .2, .9, .8, .8
        nodes.points = [Point(x=float(x), y=float(y), z=.1) for x, y in graph.xy]
        edges.id, edges.type, edges.scale.x = 1, Marker.LINE_LIST, .012
        edges.color.r, edges.color.g, edges.color.b, edges.color.a = .2, .7, 1., .5
        for i, neighbours in enumerate(graph.edges):
            for j, _ in neighbours:
                if j > i:
                    edges.points.extend([nodes.points[i], nodes.points[j]])
        self.graph_pub.publish(MarkerArray(markers=[nodes, edges]))
        path = Path()
        path.header = nodes.header
        for x, y in self.path:
            pose = PoseStamped()
            pose.header = nodes.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        self.path_pub.publish(path)

    def control(self):
        command = np.zeros(3)
        now = self.get_clock().now().nanoseconds*1e-9
        fresh = min(self.last_odom, self.last_map, self.last_sensor) > time.monotonic()-1.5
        if self.pose is not None and self.occupancy is not None and fresh and self.state not in ('returned', 'failed'):
            if not self.visited or np.linalg.norm(self.pose[:2]-self.visited[-1]) > .3:
                self.visited.append(self.pose[:2].copy())
            if now-self.last_plan >= 1.:
                self.plan(now)
                self.last_plan = now
            if len(self.path) and self.state != 'returned':
                self.wait_started = None
                # Advance past already-reached waypoints without steering backwards.
                while len(self.path) > 1 and np.linalg.norm(self.path[0]-self.pose[:2]) < .3:
                    self.path = self.path[1:]
                command = follow_path(self.pose, self.path)
            elif self.state != 'returned':
                if self.wait_started is None:
                    self.wait_started = now
                if now-self.wait_started >= 4.:
                    self.state = 'failed'
                    status = dict(time_s=now, state='failed', reason='unsupported_motion_timeout',
                                  pose=self.pose.tolist(), path=[])
                    self.trace.append(status)
                    self.status.publish(String(data=json.dumps(status)))
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = map(float, command)
        self.velocity.publish(message)
        self.command_count += 1

    def save(self):
        (self.output/'trace.json').write_text(json.dumps(self.trace, indent=2)+'\n')
        summary = dict(state=self.state, radiation_samples=self.samples, command_publications=self.command_count,
            max_graph_nodes=self.max_nodes, max_graph_edges=self.max_edges, dose_budget_usv=self.get_parameter('dose_budget').value,
            radiation_weight=self.get_parameter('radiation_weight').value, measured_dose_usv=self.dose,
            planning_p95_ms=float(np.percentile(self.planning_ms, 95)) if self.planning_ms else None,
            final_home_distance_m=float(np.linalg.norm(self.pose[:2]-self.home)) if self.pose is not None else None)
        (self.output/'exploration.json').write_text(json.dumps(summary, indent=2)+'\n')
        np.savez_compressed(self.output/'field_samples.npz', xy=np.asarray(self.field.xy).reshape(-1, 2),
                            log_rate=self.field.y)


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
