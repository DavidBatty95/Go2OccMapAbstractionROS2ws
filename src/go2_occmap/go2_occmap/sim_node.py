"""200 Hz free-body physics, policy execution and ROS 2 observation interfaces."""
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import TransformBroadcaster

from .control import Watchdog, scan_gate
from .physics import JOINTS, Simulation, rotation, true_field, truth_grid


class Simulator(Node):
    def __init__(self):
        super().__init__('go2_simulator')
        for name, default in [('world', 'partitioned'), ('duration', 120.), ('speed', 1.),
                              ('seed', 1), ('output', '/workspace/results/latest'), ('gui', False)]:
            self.declare_parameter(name, default)
        self.world = self.get_parameter('world').value
        self.sim = Simulation(self.world)
        self.output = Path(self.get_parameter('output').value)
        self.output.mkdir(parents=True, exist_ok=True)
        self.watchdog, self.ranges = Watchdog(), []
        self.finished = False
        self.create_subscription(Twist, '/cmd_vel', self.receive, 10)
        self.create_subscription(String, '/mission/status', self.mission, 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 30)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.rad_pub = self.create_publisher(Float64MultiArray, '/radiation/measurement', 20)
        self.tf = TransformBroadcaster(self)
        self.rng = np.random.default_rng(self.get_parameter('seed').value)
        self.actual_dose = self.measured_dose = self.window_counts = 0.
        self.contacts = self.scan_stops = self.command_messages = 0
        self.minimum_upright = 1.
        self.times, self.qpos, self.measured, self.doses = [], [], [], []

    def receive(self, message):
        self.command_messages += 1
        self.watchdog.update([message.linear.x, message.linear.y, message.angular.z], time.monotonic())

    def mission(self, message):
        state = json.loads(message.data).get('state')
        if state in ('returned', 'failed'):
            self.finished = state

    def stamp(self):
        ns = round(self.sim.data.time*1e9)
        return Time(sec=ns//10**9, nanosec=ns % 10**9)

    def publish_state(self):
        sim, stamp = self.sim, self.stamp()
        self.clock_pub.publish(Clock(clock=stamp))
        x, y, yaw = sim.pose
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(x), float(y)
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        vx, vy = sim.data.qvel[:2]
        odom.twist.twist.linear.x = float(math.cos(yaw)*vx+math.sin(yaw)*vy)
        odom.twist.twist.linear.y = float(-math.sin(yaw)*vx+math.cos(yaw)*vy)
        odom.twist.twist.angular.z = float((rotation(sim.data.qpos[3:7])@sim.data.qvel[3:6])[2])
        self.odom_pub.publish(odom)
        base = TransformStamped()
        base.header = odom.header
        base.child_frame_id = 'base_footprint'
        base.transform.translation.x, base.transform.translation.y = float(x), float(y)
        base.transform.rotation = odom.pose.pose.orientation
        laser = TransformStamped()
        laser.header.stamp, laser.header.frame_id, laser.child_frame_id = stamp, 'base_footprint', 'laser'
        laser.transform.translation.z, laser.transform.rotation.w = float(sim.data.qpos[2]+.12), 1.
        body = TransformStamped()
        body.header = odom.header
        body.child_frame_id = 'base_link'
        body.transform.translation.x, body.transform.translation.y, body.transform.translation.z = map(float, sim.data.qpos[:3])
        body.transform.rotation.w, body.transform.rotation.x, body.transform.rotation.y, body.transform.rotation.z = map(float, sim.data.qpos[3:7])
        self.tf.sendTransform([base, laser, body])
        joints = JointState()
        joints.header.stamp, joints.name = stamp, list(JOINTS)
        joints.position = sim.data.qpos[sim.qadr].tolist()
        joints.velocity = sim.data.qvel[sim.vadr].tolist()
        joints.effort = sim.data.ctrl[sim.actuators].tolist()
        self.joint_pub.publish(joints)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'base_link'
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = map(float, sim.data.qpos[3:7])
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, sim.data.sensor('imu_gyro').data)
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = map(float, sim.data.sensor('imu_acc').data)
        self.imu_pub.publish(imu)

    def publish_scan(self):
        self.ranges = self.sim.scan()
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = self.stamp(), 'laser'
        scan.angle_min = -math.pi
        scan.angle_increment = 2*math.pi/len(self.ranges)
        scan.angle_max = scan.angle_min+(len(self.ranges)-1)*scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time = .05, 3.5, .1
        scan.ranges = self.ranges.tolist()
        self.scan_pub.publish(scan)

    def measure(self):
        rate = float(true_field(self.sim.data.qpos[:2]))
        self.actual_dose += rate*.005/3600
        self.window_counts += rate*.005*50000/3600
        if self.sim.counter % 100 == 0:
            counts = int(self.rng.poisson(self.window_counts))
            self.window_counts = 0.
            self.measured_dose += counts/50000
            measured_rate = counts*3600/(50000*.5)
            values = [float(self.sim.data.time), *self.sim.data.qpos[:2].tolist(), measured_rate, self.measured_dose]
            self.rad_pub.publish(Float64MultiArray(data=values))
            self.measured.append(values)

    def run(self):
        duration, speed = self.get_parameter('duration').value, self.get_parameter('speed').value
        if duration <= 0 or not .1 <= speed <= 8:
            raise ValueError('duration must be positive and speed between 0.1 and 8')
        viewer = None
        if self.get_parameter('gui').value:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(self.sim.model, self.sim.data)
            viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 10., -50., 120.
        started, error = time.monotonic(), None
        reason = 'duration'
        self.get_logger().info('Go2 ready: physical joints, ideal odometry/rays; planner receives measurements only')
        try:
            while rclpy.ok() and self.sim.data.time < duration:
                rclpy.spin_once(self, timeout_sec=0.)
                if self.finished:
                    reason = self.finished
                    break
                command = self.watchdog.value(time.monotonic())
                gated = scan_gate(command, self.ranges)
                self.scan_stops += int(command[0] > 0 and gated[0] == 0)
                self.sim.step(gated)
                self.measure()
                self.contacts += int(self.sim.obstacle_contact())
                self.minimum_upright = min(self.minimum_upright, self.sim.upright)
                if self.sim.counter % 4 == 0:
                    self.publish_state()
                if self.sim.counter % 20 == 0:
                    self.publish_scan()
                    self.times.append(float(self.sim.data.time))
                    self.qpos.append(self.sim.data.qpos.copy())
                    self.doses.append(self.actual_dose)
                    if viewer:
                        viewer.cam.lookat[:] = [0., 0., .2]
                        viewer.sync()
                        if not viewer.is_running():
                            reason = 'viewer_closed'
                            break
                delay = started+self.sim.data.time/speed-time.monotonic()
                if delay > 0:
                    time.sleep(delay)
        except KeyboardInterrupt:
            reason = 'interrupted'
        except Exception as exc:
            error, reason = str(exc), 'error'
        finally:
            if viewer:
                viewer.close()
            qpos = np.asarray(self.qpos).reshape(-1, self.sim.model.nq)
            np.savez_compressed(self.output/'physics.npz', time=self.times, qpos=qpos, dose=self.doses,
                                true_occupied=truth_grid(self.world))
            np.savez_compressed(self.output/'measurements.npz', values=np.asarray(self.measured).reshape(-1, 5))
            summary = dict(world=self.world, simulation_time_s=float(self.sim.data.time),
                wall_time_s=time.monotonic()-started, reason=reason, error=error,
                actual_dose_usv=self.actual_dose, measured_dose_usv=self.measured_dose,
                obstacle_contact_steps=self.contacts, scan_stop_steps=self.scan_stops,
                minimum_upright=self.minimum_upright, command_messages=self.command_messages,
                path_length_m=float(np.linalg.norm(np.diff(qpos[:, :2], axis=0), axis=1).sum()),
                final_position=self.sim.data.qpos[:3].tolist(), seed=self.get_parameter('seed').value,
                odometry='MuJoCo ground truth', scan='ideal horizontal raycast', physics_hz=200, policy_hz=50)
            (self.output/'simulation.json').write_text(json.dumps(summary, indent=2)+'\n')
            self.get_logger().info(json.dumps(summary))
        return 1 if error else 0


def main():
    rclpy.init()
    node = Simulator()
    try:
        result = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(result)
